#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
#define rr 0.0625
// distance from back wheel to center in meters
#define bb 0.119

void followTrajectory(double goalX, double goalY, double heading, double finalheading, double desiredheading, String scenario, bool resetState);
void updateOdometry(double heading);

#endif