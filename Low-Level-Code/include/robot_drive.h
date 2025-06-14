#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#define NUM_MOTORS 4

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define pidTau 0.1

#define MAX_FORWARD 5
#define MAX_TURN 3
#define MAX_GRABBER 1

void setupDrive();
void updateSetpoints(double left, double right, double up);
void updateJoystickSetpoints(double left, double right, double up);
void resetGripper();
void useGripper(float grip);
void updatePIDs();

#endif // ROBOT_DRIVE_H