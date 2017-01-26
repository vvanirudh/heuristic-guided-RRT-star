# Heuristic Guided RRT*

Please refer to the technical report [Learning Motion Planning Assumptions](http://www.ri.cmu.edu/pub_files/2014/8/LearningMotionPlanningAssumptions.pdf) for a theoretical description of the algorithm.

## Description
This planner extends RRTStar from OMPL and integrates a coarse grid-based value function obtained from a discrete planner (maybe djikstra) into the RRTStar to vary its sampling strategy to bias towards regions of low value function.

## Author
Anirudh Vemula
