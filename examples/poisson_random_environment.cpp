/**
 * Created on: June 12th, 2014
 * Author: Anirudh Vemula
 */

#include <ros/ros.h>
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/visualization/markers.h"
#include "ompl/geometric/PathSimplifier.h"
#include "environment_generator/environments.h"
#include "RRTCoarse/RRTCoarse.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ov = ompl::visualization;

using namespace ca;
using namespace env;
using namespace shapes;

bool isStateValid(boost::shared_ptr<Environment> env, const ob::SpaceInformation *si, const ob::State *state) {
  const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
  double x=s->getX(), y=s->getY();
  return si->satisfiesBounds(s) && !env->InCollision(x, y);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si) {
  return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "poisson_random_environment");
  ros::NodeHandle n("~");

  ros::Publisher pub_path_marker = n.advertise<visualization_msgs::Marker>("path", 1);
  ros::Publisher pub_graph_marker = n.advertise<visualization_msgs::Marker>("graph", 1);
  ros::Publisher pub_env_marker_array = n.advertise<visualization_msgs::MarkerArray>("environment", 1);
  ros::Duration(1.0).sleep();

  PoissonRandomEnvironment Poissonenv;
  Poissonenv.Initialize(0.005, 0, 0, 100, 100);
  Poissonenv.CarveCircle(1,1,5);
  Poissonenv.CarveCircle(99,99,5);
  boost::shared_ptr<Environment> env = boost::shared_ptr<Environment>(new PoissonRandomEnvironment(Poissonenv));

  ob::StateSpacePtr space(new ob::DubinsStateSpace(2.0, false));

  ob::ScopedState<> start(space), goal(space);
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(100);
  space->as<ob::SE2StateSpace>()->setBounds(bounds);

  ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
  si->setStateValidityChecker(boost::bind(&isStateValid, env, si.get(), _1));
  si->setStateValidityCheckingResolution(0.002);

  start[0] = start[1] = 1.; start[2] = 0.;
  goal[0] = goal[1] = 99; goal[2] = 0*-.99*boost::math::constants::pi<double>();

  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal);
  pdef->setOptimizationObjective(getPathLengthObjective(si));

  boost::shared_ptr<RRTCoarse> rrtc = boost::shared_ptr<RRTCoarse>(new RRTCoarse(si));
  rrtc->setRange(2);

  ob::PlannerPtr optimizingPlanner(rrtc);
  optimizingPlanner->setProblemDefinition(pdef);
  optimizingPlanner->setup();

  visualization_msgs::MarkerArray ma = env->GetMarkerArray(0, 0, 1, 0.3);
  for (std::size_t i = 0; i < ma.markers.size(); i++)
    ma.markers[i].id = i;
  pub_env_marker_array.publish(ma);

  ob::PlannerStatus solved = optimizingPlanner->solve(1.0);

  if (solved) {
    std::vector<double> reals;

    ROS_INFO_STREAM("Found solution!");

    boost::shared_ptr<og::PathGeometric> path = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
    path->interpolate(1000);
    visualization_msgs::Marker path_marker = ov::GetMarker(*path, 0.1);
    path_marker.ns = "path";
    path_marker.header.stamp = ros::Time::now();
    path_marker.header.frame_id = "/world";
    pub_path_marker.publish(path_marker);

    ob::PlannerData data(si);
    optimizingPlanner->getPlannerData(data);
    visualization_msgs::Marker graph_marker = ov::GetMarker(data, 100, 0.05, 0, 0, 1, 0.3);
    graph_marker.ns = "graph";
    graph_marker.header.stamp = ros::Time::now();
    graph_marker.header.frame_id = "/world";
    pub_graph_marker.publish(graph_marker);
  }
  else {
    ROS_INFO_STREAM("No solution found");
    ob::PlannerData data(si);
    optimizingPlanner->getPlannerData(data);
    visualization_msgs::Marker graph_marker = ov::GetMarker(data, 100, 0.05, 0, 0, 1, 0.3);
    graph_marker.ns = "graph";
    graph_marker.header.stamp = ros::Time::now();
    graph_marker.header.frame_id = "/world";
    pub_graph_marker.publish(graph_marker);

  }
  ros::Duration(1.0).sleep();
}