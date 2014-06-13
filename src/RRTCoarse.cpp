/**
* Created : June 11th
* Author : Anirudh Vemula
*/
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "RRTCoarse/RRTCoarse.h"
#include <ompl/base/StateSpaceTypes.h>
#include <algorithm>
#include <limits>
#include <map>
#include <queue>
#include <math.h>
#include <boost/math/constants/constants.hpp>
#include <iostream>

/**
 * \brief Overloading(or defining) the less than operator for grid cells. 
 * This is done in the reverse way so that the cell with the least value
 * will float upto the top of the priority queue
 */
struct CellComparator {
    bool operator() (const Grid<int>::Cell* lhs, const Grid<int>::Cell* rhs) {
        return lhs->data > rhs->data;
    }
};
// bool operator<(const Grid<int>::Cell& lhs, const Grid<int>::Cell& rhs) {
//     if(lhs.data>rhs.data)
//         return true;
//     else 
//         return false;
// }

RRTCoarse::RRTCoarse(const base::SpaceInformationPtr &si) : base::Planner(si, "RRTCoarse"), grid_(2)
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    goalBias_ = 0.05;
    maxDistance_ = 0.0;
    delayCC_ = true;
    lastGoalMotion_ = NULL;

    /* My addition */
    exploreBias_ = 0.1;
    /* End of my addition */

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

    Planner::declareParam<double>("range", this, &RRTCoarse::setRange, &RRTCoarse::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTCoarse::setGoalBias, &RRTCoarse::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTCoarse::setDelayCC, &RRTCoarse::getDelayCC, "0,1");

    addPlannerProgressProperty("iterations INTEGER",
                               boost::bind(&RRTCoarse::getIterationCount, this));
    addPlannerProgressProperty("collision checks INTEGER",
                               boost::bind(&RRTCoarse::getCollisionCheckCount, this));
    addPlannerProgressProperty("best cost REAL",
                               boost::bind(&RRTCoarse::getBestCost, this));
}

RRTCoarse::~RRTCoarse(void)
{
    freeMemory();
}

void RRTCoarse::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&RRTCoarse::distanceFunction, this, _1, _2));


    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
        OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed planning time.", getName().c_str());
        opt_.reset(new base::PathLengthOptimizationObjective(si_));
    }
}

void RRTCoarse::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    lastGoalMotion_ = NULL;
    goalMotions_.clear();

    iterations_ = 0;
    collisionChecks_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());

}

ompl::base::PlannerStatus RRTCoarse::solve(const base::PlannerTerminationCondition &ptc)
{
    /* My addition */
    buildGrid();
    std::priority_queue<Grid<int>::Cell*, std::vector<Grid<int>::Cell*>, CellComparator> cellQueue;
    /* End of my addition */
    
    checkValidity();
    base::Goal                  *goal   = pdef_->getGoal().get();
    base::GoalSampleableRegion  *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    bool symDist = si_->getStateSpace()->hasSymmetricDistance();
    bool symInterp = si_->getStateSpace()->hasSymmetricInterpolate();
    bool symCost = opt_->isSymmetric();

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->cost = opt_->identityCost();
        nn_->add(motion);
        /* My addition */
        if(si_->getStateSpace()->getType()==base::STATE_SPACE_SE2) {
            /* Debug Statements */
            const base::SE2StateSpace::StateType* debug_st = st->as<base::SE2StateSpace::StateType>();
            double debug_x = debug_st->getX();
            double debug_y = debug_st->getY();
            std::cout<<"Added start states "<<debug_x<<" "<<debug_y<<std::endl;
            /* End */
            cellQueue.push((getGridCell(st)));
        }
        /* End of my addition */
    }
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting with %u states", getName().c_str(), nn_->size());

    Motion *solution       = lastGoalMotion_;

    // \TODO Make this variable unnecessary, or at least have it
    // persist across solve runs
    base::Cost bestCost    = opt_->infiniteCost();

    Motion *approximation  = NULL;
    double approximatedist = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    Motion *rmotion        = new Motion(si_);
    base::State *rstate    = rmotion->state;
    base::State *xstate    = si_->allocState();

    // e+e/d.  K-nearest RRT*
    double k_rrg           = boost::math::constants::e<double>() + (boost::math::constants::e<double>()/(double)si_->getStateSpace()->getDimension());

    std::vector<Motion*>       nbh;

    std::vector<base::Cost>    costs;
    std::vector<base::Cost>    incCosts;
    std::vector<std::size_t>   sortedCostIndices;

    std::vector<int>           valid;
    unsigned int               rewireTest = 0;
    unsigned int               statesGenerated = 0;

    if (solution)
        OMPL_INFORM("%s: Starting with existing solution of cost %.5f", getName().c_str(), solution->cost.v);
    OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(), (unsigned int)std::ceil(k_rrg * log((double)(nn_->size()+1))));


    // our functor for sorting nearest neighbors
    CostIndexCompare compareFn(costs, *opt_);
    std::cout<<"PTC : "<<ptc<<std::endl;

    while (ptc == false)
    {
        iterations_++;
        std::cout<<"Iterations : "<<iterations_<<std::endl;
        if (goal_s && goalMotions_.size() < goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
            /* Debug Statements */
            std::cout<<"Sampled Goal"<<std::endl;
            /* End */
            goal_s->sampleGoal(rstate);
        }
        else if(rng_.uniform01() < exploreBias_) {
            /* Debug Statements */
            std::cout<<"Sampled Randomly"<<std::endl;
            /* End */
            sampler_->sampleUniform(rstate);
        }
        else {
            Grid<int>::Cell *cell = cellQueue.top();
            double xCoord = cell->coord[0];
            double yCoord = cell->coord[1];
            /* Debug Statements */
            std::cout<<"Popped cell coordinates "<<xCoord<<" "<<yCoord<<std::endl;
            /* End */
            base::State* tempSt = si_->allocState();
            base::SE2StateSpace::StateType* nearState = tempSt->as<base::SE2StateSpace::StateType>();
            nearState->setX(xCoord);
            nearState->setY(yCoord);
            base::State* nearSt = nearState;
            sampler_->sampleUniformNear(nearSt, rstate, maxDistance_);
            /* Debug Statements */
            std::cout<<"Sampled Point "<<(rstate->as<base::SE2StateSpace::StateType>())->getX()<<" "<<(rstate->as<base::SE2StateSpace::StateType>())->getY()<<std::endl;
            /* End */
        }
        // }

        // find closest state in the tree
        Motion *nmotion = nn_->nearest(rmotion);

        base::State *dstate = rstate;

        // find state to add to the tree
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        // Check if the motion between the nearest state and the state to add is valid
        ++collisionChecks_;
        if (si_->checkMotion(nmotion->state, dstate))
        {
            // create a motion
            Motion *motion = new Motion(si_);
            si_->copyState(motion->state, dstate);
            motion->parent = nmotion;
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost =  opt_->combineCosts(nmotion->cost, motion->incCost);
            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM every other point TO our new point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
            if (!symDist)
                nn_->setDistanceFunction(boost::bind(&RRTCoarse::distanceFunction, this, _1, _2));

            // Find nearby neighbors of the new motion - k-nearest RRT*
            unsigned int k = std::ceil(k_rrg * log((double)(nn_->size()+1)));
            nn_->nearestK(motion, k, nbh);
            rewireTest += nbh.size();
            statesGenerated++;

            // cache for distance computations
            //
            // Our cost caches only increase in size, so they're only
            // resized if they can't fit the current neighborhood
            if (costs.size() < nbh.size())
            {
                costs.resize(nbh.size());
                incCosts.resize(nbh.size());
                sortedCostIndices.resize(nbh.size());
            }

            // cache for motion validity (only useful in a symmetric space)
            //
            // Our validity caches only increase in size, so they're
            // only resized if they can't fit the current neighborhood
            if (symDist && symInterp)
            {
                if (valid.size() < nbh.size())
                    valid.resize(nbh.size());
                std::fill(valid.begin(), valid.begin()+nbh.size(), 0);
            }

            // Finding the nearest neighbor to connect to
            // By default, neighborhood states are sorted by cost, and collision checking
            // is performed in increasing order of cost
            if (delayCC_)
            {
                // calculate all costs and distances
                for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                    costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                }

                // sort the nodes
                //
                // we're using index-value pairs so that we can get at
                // original, unsorted indices
                for (std::size_t i = 0; i < nbh.size(); ++i)
                    sortedCostIndices[i] = i;
                std::sort(sortedCostIndices.begin(), sortedCostIndices.begin()+nbh.size(),
                          compareFn);

                // collision check until a valid motion is found
                for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
                     i != sortedCostIndices.begin()+nbh.size();
                     ++i)
                {
                    if (nbh[*i] != nmotion)
                        ++collisionChecks_;
                    if (nbh[*i] == nmotion || si_->checkMotion(nbh[*i]->state, motion->state))
                    {
                        motion->incCost = incCosts[*i];
                        motion->cost = costs[*i];
                        motion->parent = nbh[*i];
                        if (symDist && symInterp)
                            valid[*i] = 1;
                        break;
                    }
                    else if (symDist && symInterp)
                        valid[*i] = -1;
                }
            }
            else // if not delayCC
            {
                motion->incCost = opt_->motionCost(nmotion->state, motion->state);
                motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
                // find which one we connect the new state to
                for (std::size_t i = 0 ; i < nbh.size(); ++i)
                {
                    if (nbh[i] != nmotion)
                    {
                        incCosts[i] = opt_->motionCost(nbh[i]->state, motion->state);
                        costs[i] = opt_->combineCosts(nbh[i]->cost, incCosts[i]);
                        if (opt_->isCostBetterThan(costs[i], motion->cost))
                        {
                            ++collisionChecks_;
                            if (si_->checkMotion(nbh[i]->state, motion->state))
                            {
                                motion->incCost = incCosts[i];
                                motion->cost = costs[i];
                                motion->parent = nbh[i];
                                if (symDist && symInterp)
                                    valid[i] = 1;
                            }
                            else if (symDist && symInterp)
                                valid[i] = -1;
                        }
                    }
                    else
                    {
                        incCosts[i] = motion->incCost;
                        costs[i] = motion->cost;
                        if (symDist && symInterp)
                            valid[i] = 1;
                    }
                }
            }

            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            /* My addition */
            if(si_->getStateSpace()->getType()==base::STATE_SPACE_SE2) {
                // cellsExplored.push_back(getGridCell(dstate));
                /* Debug Statements */
                std::cout<<"Added state "<<(dstate->as<base::SE2StateSpace::StateType>())->getX() << " " << (dstate->as<base::SE2StateSpace::StateType>())->getY()<<std::endl;
                /* End */
                cellQueue.push((getGridCell(dstate)));
                /* Debug Statements */
                std::cout<<"Pushed Cell "<<getGridCell(dstate)->coord[0]<<" "<<getGridCell(dstate)->coord[1]<<std::endl;
                /* End */
            }
            /* End of my addition */

            bool checkForSolution = false;
            // rewire tree if needed
            //
            // This sounds crazy but for asymmetric distance functions this is necessary
            // For this case, it has to be FROM our new point TO each other point
            // NOTE THE ORDER OF THE boost::bind PARAMETERS
            if (!symDist)
            {
                nn_->setDistanceFunction(boost::bind(&RRTCoarse::distanceFunction, this, _2, _1));
                nn_->nearestK(motion, k, nbh);
                rewireTest += nbh.size();
            }

            for (std::size_t i = 0; i < nbh.size(); ++i)
            {
                if (nbh[i] != motion->parent)
                {
                    base::Cost nbhIncCost;
                    if (symDist && symCost)
                        nbhIncCost = incCosts[i];
                    else
                        nbhIncCost = opt_->motionCost(motion->state, nbh[i]->state);
                    base::Cost backupCost = motion->cost;
                    base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
                    base::Cost oldnbhCost =  nbh[i]->cost;
                    //double motion_x = motion->state->as<ompl::base::DubinsStateSpace::StateType>()->getX();
                    //double motion_y = motion->state->as<ompl::base::DubinsStateSpace::StateType>()->getY();
                    base::Cost oldnbhParentCost;
                    if (nbh[i]->parent)
                      oldnbhParentCost = nbh[i]->parent->cost;
                    base::Cost child1Cost, child2Cost;
                    if (nbh[i]->children.size()>0)
                      child1Cost = nbh[i]->children[0]->cost;

                    if (opt_->isCostBetterThan(nbhNewCost, nbh[i]->cost))
                    {
                        bool motionValid;
                        if (symDist && symInterp)
                        {
                            if (valid[i] == 0)
                            {
                                ++collisionChecks_;
                                motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                            }
                            else
                                motionValid = (valid[i] == 1);
                        }
                        else
                        {
                            ++collisionChecks_;
                            motionValid = si_->checkMotion(motion->state, nbh[i]->state);
                        }
                        if (motionValid)
                        {
                            // Remove this node from its parent list
                            removeFromParent (nbh[i]);

                            // Add this node to the new parent
                            nbh[i]->parent = motion;
                            nbh[i]->incCost = nbhIncCost;
                            nbh[i]->cost = nbhNewCost;
                            nbh[i]->parent->children.push_back(nbh[i]);

                            // Update the costs of the node's children
                            updateChildCosts(nbh[i]);

                            checkForSolution = true;
                        }
                    }
                }
            }

            // Add the new motion to the goalMotion_ list, if it satisfies the goal
            double distanceFromGoal;
            if (goal->isSatisfied(motion->state, &distanceFromGoal))
            {
                goalMotions_.push_back(motion);
                checkForSolution = true;
            }

            // Checking for solution or iterative improvement
            if (checkForSolution)
            {
                for (size_t i = 0; i < goalMotions_.size(); ++i)
                {
                    if (opt_->isCostBetterThan(goalMotions_[i]->cost, bestCost))
                    {
                        bestCost = goalMotions_[i]->cost;
                        bestCost_ = bestCost;
                    }

                    sufficientlyShort = opt_->isSatisfied(goalMotions_[i]->cost);
                    if (sufficientlyShort)
                    {
                        solution = goalMotions_[i];
                        break;
                    }
                    else if (!solution ||
                             opt_->isCostBetterThan(goalMotions_[i]->cost,solution->cost))
                        solution = goalMotions_[i];
                }
            }

            // Checking for approximate solution (closest state found to the goal)
            if (goalMotions_.size() == 0 && distanceFromGoal < approximatedist)
            {
                approximation = motion;
                approximatedist = distanceFromGoal;
            }
        }

        // terminate if a sufficient solution is found
        if (solution && sufficientlyShort)
            break;
    }

    bool approximate = (solution == 0);
    bool addedSolution = false;
    if (approximate)
        solution = approximation;
    else
        lastGoalMotion_ = solution;

    if (solution != 0)
    {
        // construct the solution path
        std::vector<Motion*> mpath;
        while (solution != 0)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        // set the solution path
        PathGeometric *geoPath = new PathGeometric(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            geoPath->append(mpath[i]->state);

        base::PathPtr path(geoPath);
        // Add the solution path, whether it is approximate (not reaching the goal), and the
        // distance from the end of the path to the goal (-1 if satisfying the goal).
        base::PlannerSolution psol(path, approximate, approximate ? approximatedist : -1.0);
        // Does the solution satisfy the optimization objective?
        psol.optimized_ = sufficientlyShort;

        pdef_->addSolutionPath (psol);

        addedSolution = true;
    }

    si_->freeState(xstate);
    if (rmotion->state)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree.", getName().c_str(), statesGenerated, rewireTest, goalMotions_.size());

    return base::PlannerStatus(addedSolution, approximate);
}

void RRTCoarse::removeFromParent(Motion *m)
{
    std::vector<Motion*>::iterator it = m->parent->children.begin ();
    while (it != m->parent->children.end ())
    {
        if (*it == m)
        {
            it = m->parent->children.erase(it);
            it = m->parent->children.end ();
        }
        else
            ++it;
    }
}

void RRTCoarse::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
      base::Cost tempc1 = m->children[i]->cost;
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        base::Cost tempc2 = m->children[i]->cost;
        updateChildCosts(m->children[i]);
    }
}

void RRTCoarse::freeMemory(void)
{
    if (nn_)
    {
        std::vector<Motion*> motions;
        nn_->list(motions);
        for (std::size_t i = 0 ; i < motions.size() ; ++i)
        {
            if (motions[i]->state)
                si_->freeState(motions[i]->state);
            delete motions[i];
        }
    }
}

void RRTCoarse::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion*> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (std::size_t i = 0 ; i < motions.size() ; ++i)
    {
        if (motions[i]->parent == NULL)
            data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
        else
            data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                         base::PlannerDataVertex(motions[i]->state));
    }
    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
    data.properties["collision_checks INTEGER"] =
        boost::lexical_cast<std::string>(collisionChecks_);
}

std::string RRTCoarse::getIterationCount(void) const
{
  return boost::lexical_cast<std::string>(iterations_);
}
std::string RRTCoarse::getCollisionCheckCount(void) const
{
  return boost::lexical_cast<std::string>(collisionChecks_);
}
std::string RRTCoarse::getBestCost(void) const
{
  return boost::lexical_cast<std::string>(bestCost_.v);
}


void RRTCoarse::buildGrid(void) {
    if(si_->getStateSpace()->getType()!=base::STATE_SPACE_SE2) {
        return;
    }

    /* State space is SE2 */
    base::Goal *goal = pdef_->getGoal().get();
    // base::GoalState *goalstateptr = dynamic_cast<base::GoalState*>(goal);
    base::GoalState* goalstateptr = goal->as<base::GoalState>();
    base::State *goalstate = goalstateptr->getState(); // Obtained the goal state

    /* Get the goal coordinates */
    // double goalX = dynamic_cast<base::SE2StateSpace::StateType*>(goalstate)->getX();
    // double goalY = dynamic_cast<base::SE2StateSpace::StateType*>(goalstate)->getY();
    base::SE2StateSpace::StateType * goalst = goalstate->as<base::SE2StateSpace::StateType>();
    double goalX = goalst->getX();
    double goalY = goalst->getY();

    /* Get the start coordinates */
    // double startX = dynamic_cast<base::SE2StateSpace::StateType*>(pdef_->getStartState(0))->getX();
    // double startY = dynamic_cast<base::SE2StateSpace::StateType*>(pdef_->getStartState(0))->getY();
    base::SE2StateSpace::StateType* startst = pdef_->getStartState(0)->as<base::SE2StateSpace::StateType>();
    double startX = startst->getX();
    double startY = startst->getY();

    /* Get bounds of the state space */
    boost::shared_ptr<base::StateSpace> space = si_->getStateSpace();
    // base::RealVectorBounds bounds = dynamic_cast<base::SE2StateSpace*>(space.get())->getBounds();
    base::SE2StateSpace* statespace = (space.get())->as<base::SE2StateSpace>();
    base::RealVectorBounds bounds = statespace->getBounds();
    double xlow = bounds.low[0];
    double xhigh = bounds.high[0];
    double ylow = bounds.low[1];
    double yhigh = bounds.high[1];

    /* Create all grid cells */
    for(int i = std::floor(xlow); i < std::floor(xhigh) + 1; i++) {
        for(int j = std::floor(ylow); j < std::floor(yhigh) + 1; j++) {
            std::vector<int> coord;
            coord.push_back(i);
            coord.push_back(j);
            Grid<int>::Cell * cell = grid_.createCell(coord);
            cell->data = -1;
            grid_.add(cell);
        }
    }

    /* Run Djikstra/ some other to assign value function to all cells */
    std::queue<Grid<int>::Cell*> cellqueue;
    std::vector<int> coord;
    coord.push_back(std::floor(goalX));
    coord.push_back(std::floor(goalY));
    Grid<int>::Cell * goal_cell = grid_.getCell(coord);
    goal_cell->data = 0;
    cellqueue.push(goal_cell);
    while(!cellqueue.empty()) {
        Grid<int>::Cell* cell = cellqueue.front();
        cellqueue.pop();
        coord = cell->coord;
        int x = coord[0];
        int y = coord[1];
        int data = cell->data;
        data++;
        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                if(i==0 && j==0)
                    continue;
                coord[0] = x+i;
                coord[1] = y+j;
                if(grid_.has(coord)) {
                    cell = grid_.getCell(coord);
                    if(cell->data==-1) {
                        if(isValidCoord(coord)) {
                            cell->data = data;
                            cellqueue.push(cell);
                        }
                        else {
                            /* Debug Statements */
                            std::cout<<"Cell "<<coord[0]<<" "<<coord[1]<<" has obstacle"<<std::endl;
                            /* End */
                            cell->data = std::numeric_limits<int>::max();
                        }
                    }
                }
            }
        }
    }
}

bool RRTCoarse::isValidCoord(std::vector<int> coord) {
    int x = coord[0];
    int y = coord[1];
    // base::SE2StateSpace::StateType state;
    base::State* state = si_->allocState();
    base::SE2StateSpace::StateType* st = state->as<base::SE2StateSpace::StateType>();
    /* Check the end points for validity */
    for(int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            st->setXY(x+i,y+j);
            base::State * s = st;
            if(!si_->isValid(s))
                return false;
        }
    }
    /* Check the center of the cell for validity */
    st->setXY(x+0.5,y+0.5);
    base::State * s = st;
    if(!si_->isValid(s))
        return false;
    return true;
}

Grid<int>::Cell* RRTCoarse::getGridCell(const base::State * s) {
    const base::SE2StateSpace::StateType * st = s->as<base::SE2StateSpace::StateType>();
    double stateX = st->getX();
    double stateY = st->getY();

    int x = std::floor(stateX);
    int y = std::floor(stateY);
    std::vector<int> coord;
    coord.push_back(x);
    coord.push_back(y);
    return grid_.getCell(coord);
}