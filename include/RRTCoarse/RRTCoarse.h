/**
* Created : June 11th
* Author : Anirudh Vemula
*/

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/base/ProjectionEvaluator.h>
#include <ompl/datastructures/Grid.h>
#include <ompl/datastructures/PDF.h>
#include <limits>
#include <vector>
#include <utility>

using namespace ompl;
using namespace geometric;

class RRTCoarse : public base::Planner
{
	public:

	RRTCoarse(const base::SpaceInformationPtr &si);

	virtual ~RRTCoarse(void);

	virtual void getPlannerData(base::PlannerData &data) const;

	virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

	virtual void clear(void);

	/** \brief Set the goal bias

	    In the process of randomly selecting states in
	    the state space to attempt to go towards, the
	    algorithm may in fact choose the actual goal state, if
	    it knows it, with some probability. This probability
	    is a real number between 0.0 and 1.0; its value should
	    usually be around 0.05 and should not be too large. It
	    is probably a good idea to use the default value. */
	void setGoalBias(double goalBias)
	{
	    goalBias_ = goalBias;
	}

	/** \brief Get the goal bias the planner is using */
	double getGoalBias(void) const
	{
	    return goalBias_;
	}

	/** \brief Set the range the planner is supposed to use.

	    This parameter greatly influences the runtime of the
	    algorithm. It represents the maximum length of a
	    motion to be added in the tree of motions. */
	void setRange(double distance)
	{
	    maxDistance_ = distance;
	}

	/** \brief Get the range the planner is using */
	double getRange(void) const
	{
	    return maxDistance_;
	}

	/** \brief Set a different nearest neighbors datastructure */
	template<template<typename T> class NN>
	void setNearestNeighbors(void)
	{
	    nn_.reset(new NN<Motion*>());
	}

	/** \brief Option that delays collision checking procedures.
	    When it is enabled, all neighbors are sorted by cost. The
	    planner then goes through this list, starting with the lowest
	    cost, checking for collisions in order to find a parent. The planner
	    stops iterating through the list when a collision free parent is found.
	    This prevents the planner from collsion checking each neighbor, reducing
	    computation time in scenarios where collision checking procedures are expensive.*/
	void setDelayCC(bool delayCC)
	{
	    delayCC_ = delayCC;
	}

	/** \brief Get the state of the delayed collision checking option */
	bool getDelayCC(void) const
	{
	    return delayCC_;
	}

	virtual void setup(void);

	/* My addition */
	/**
	 * \brief Sets the exploration bias
	 * @param exploreBias The probability with which it should explore
	 */
	void setExploreBias(double exploreBias) {
		exploreBias_ = exploreBias;
	}

	/**
	 * \brief Gets the exploration bias 
	 * @return  The probability with which it explores
	 */
	double getExploreBias(void) {
		return exploreBias_;
	}

	/**
	 * \brief Sets the grid resolution
	 * @param resolution The grid resolution
	 */
	void setResolution(double resolution) {
		resolution_ = resolution;
	}

	/**
	 * \brief Returns the grid resolution
	 * @return  The resolution of the grid
	 */
	double getResolution(void) {
		return resolution_;
	}
	/* End of my addition */

	///////////////////////////////////////
	// Planner progress property functions
	std::string getIterationCount(void) const;

	std::string getCollisionCheckCount(void) const;

	std::string getBestCost(void) const;
	///////////////////////////////////////

	protected:

	/** \brief Representation of a motion */
	class Motion
	{
	public:
	    /** \brief Constructor that allocates memory for the state. This constructor automatically allocates memory for \e state, \e cost, and \e incCost */
	    Motion(const base::SpaceInformationPtr &si) :
	        state(si->allocState()),
	        parent(NULL)
	    {
	    }

	    ~Motion(void)
	    {
	    }

	    /** \brief The state contained by the motion */
	    base::State       *state;

	    /** \brief The parent motion in the exploration tree */
	    Motion            *parent;

	    /** \brief The cost up to this motion */
	    base::Cost        cost;

	    /** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance computations in the updateChildCosts() method) */
	    base::Cost        incCost;

	    /** \brief The set of motions descending from the current motion */
	    std::vector<Motion*> children;
	};

	/** \brief Free the memory allocated by this planner */
	void freeMemory(void);

	// For sorting a list of costs and getting only their sorted indices
	struct CostIndexCompare
	{
	    CostIndexCompare(const std::vector<base::Cost>& costs,
	                     const base::OptimizationObjective& opt) :
	        costs_(costs), opt_(opt)
	    {}
	    bool operator()(unsigned i, unsigned j)
	    {
	        return opt_.isCostBetterThan(costs_[i],costs_[j]);
	    }
	    const std::vector<base::Cost>& costs_;
	    const base::OptimizationObjective& opt_;
	};

	/** \brief Compute distance between motions (actually distance between contained states) */
	double distanceFunction(const Motion* a, const Motion* b) const
	{
	    return si_->distance(a->state, b->state);
	}

	/** \brief Removes the given motion from the parent's child list */
	void removeFromParent(Motion *m);

	/** \brief Updates the cost of the children of this node if the cost up to this node has changed */
	void updateChildCosts(Motion *m);

	/** \brief State sampler */
	base::StateSamplerPtr                          sampler_;

	/** \brief A nearest-neighbors datastructure containing the tree of motions */
	boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

	/** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
	double                                         goalBias_;

	/** \brief The maximum length of a motion to be added to a tree */
	double                                         maxDistance_;

	/** \brief The random number generator */
	RNG                                            rng_;

	/** \brief Option to delay and reduce collision checking within iterations */
	bool                                           delayCC_;

	/** \brief Objective we're optimizing */
	base::OptimizationObjectivePtr opt_;

	/** \brief The most recent goal motion.  Used for PlannerData computation */
	Motion                                         *lastGoalMotion_;

	/** \brief A list of states in the tree that satisfy the goal condition */
	std::vector<Motion*>                           goalMotions_;

	//////////////////////////////
	// Planner progress properties

	/** \brief Number of iterations the algorithm performed */
	unsigned int                                   iterations_;

	/** \brief Number of collisions checks performed by the algorithm */
	unsigned int                                   collisionChecks_;

	/** \brief Best cost found so far by algorithm */
	base::Cost                                     bestCost_;

	/* My addition */

	/**
	 * \brief The grid on the statespace that contains the value function
	 */
	Grid<int>																				grid_;

	/**
	 * \brief The function that builds the value function grid (resolution = 1)
	 */
	void buildGrid(void);

	/**
	 * \brief A generic grid building function that takes the resolution as a parameter
	 * @param resolution The Grid resolution
	 */
	void buildGrid(double resolution);

	/**
	 * \brief The function that checks whether a given coordinate/cell contains an obstacle (resolution = 1)
	 * @param  coord vector containing the coordinate values
	 * @return       True if the cell does not contain an obstacle, false if it does.
	 */
	bool isValidCoord(std::vector<int> coord);

	/**
	 * \brief The function that checks whether a given coordinate/cell contains an obstacle (resolution version)
	 * @param  coord      Vector that contains the coordinate values
	 * @param  resolution Resolution of the grid
	 * @return            True if the cell does not contain an obstacle, false otherwise
	 */
	bool isValidCoord(std::vector<int> coord, double resolution);

	/**
	 * \brief Gives the grid cell corresponding to the state (resolution = 1)
	 * @param  s Input State
	 * @return   The cell corresponding to the given input state
	 */
	Grid<int>::Cell* getGridCell(const base::State* s);

	/**
	 * \brief Gives the  grid cell corresponding to the state (resolution version)
	 * @param  s          input state
	 * @param  resolution Grid resolution
	 * @return            The cell corresponding to the given input state
	 */
	Grid<int>::Cell* getGridCell(const base::State* s, double resolution);

	/**
	 * \brief The fraction of time some random state is sampled instead of biasing it towards the decreasing gradient of value function
	 */
	double 																					exploreBias_;

	/**
	 * \brief The grid resolution
	 */
	double 																					resolution_;	
};