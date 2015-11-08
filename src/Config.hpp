#ifndef _EXPLORATION_CONFIG_HPP_
#define _EXPLORATION_CONFIG_HPP_

namespace exploration
{

struct Weights {
    Weights() : explCells(1.0), angDist(1.0), robotGoalDist(1.0), driveability(1.0), 
            edgeDetected(1.0) {
    }
    
    Weights(double expl, double ang, double dist, double driveability, double edge) :
            explCells(expl), angDist(ang), robotGoalDist(dist), driveability(driveability), 
            edgeDetected(edge) {
    }
    
    double explCells; // Number of newly explored cells.
    double angDist; // Angle the robot has to be turned to face the goal.
    double robotGoalDist; // Distance between robot and goal.
    double driveability; // Worst driveability of the goal region.
    // If the goal pose is far away from the robot an edge detection is used to 
    // get the orientation. This value defines the bonus the exploration pose receives
    // if the detection has been successful.
    double edgeDetected;
};
    
struct Config { 
    struct Weights weights;
};

} // end namespace motion_planning_libraries

#endif
