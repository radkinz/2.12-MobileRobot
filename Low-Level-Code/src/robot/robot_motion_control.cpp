#include <Arduino.h>
#include "util.h"
#include "robot_drive.h"
#include "EncoderVelocity.h"
#include "wireless.h"
#include "robot_motion_control.h"

// #define UTURN
// #define CIRCLE
//#define JOYSTICK
//#define POINT_TO_POINT
//#define REVERSE
// #define YOUR_TRAJECTORY

extern RobotMessage robotMessage;
extern ControllerMessage controllerMessage;

int state = 0;
double robotVelocity = 0; // velocity of robot, in m/s
double k = 0;             // k is 1/radius from center of rotation circle
double upVelocity = 0;

extern EncoderVelocity encoders[NUM_MOTORS];
double currPhiL = 0;
double currPhiR = 0;
double prevPhiL = 0;
double prevPhiR = 0;

// ———————————————— PARAMETERS ————————————————————————————————————————
const double k_rho = 1.0;   // gain for distance
const double k_alpha = 2.0; // gain for heading error
const double k_beta = -0.5; // gain for final orientation error

const double turnOmega = 0.5; // rad/s for in-place turns

const double rho_tol = 0.05;   // 5 cm distance tolerance
const double alpha_tol = 0.05; // ~3° angle tolerance

// ———————————————— STATE ——————————————————————————————————————————
int currentState = 0; // 0 = drive to point, 1 = rotate to final heading, ≥2 = done

// ———————————————— HELPERS —————————————————————————————————————————
double normalizeAngle(double a)
{
  while (a > M_PI)
    a -= 2 * M_PI;
  while (a <= -M_PI)
    a += 2 * M_PI;
  return a;
}

// Sets the desired wheel velocities based on desired robot velocity in m/s
// and k curvature in 1/m representing 1/(radius of curvature)
void setWheelVelocities(float robotVelocity, float k)
{

    float upVelocity = 0;
    double left = (robotVelocity - k * bb * robotVelocity) / rr;
    double right = 2 * robotVelocity / rr - left;
    double up = upVelocity;
    updateSetpoints(left, right, upVelocity);
}

// Makes robot follow a trajectory
void followTrajectory(double goalX, double goalY, double heading, double finalheading, double desiredheading, String scenario, bool resetState) {
    if (resetState ) {
      state = 0;
    }

    if (goalX == 0 && goalY == 0 && scenario == "POINT_TO_POINT") {
        updateSetpoints(0, 0, 0);
        return;
    }

    if (scenario == "JOYSTICK") {
        if (freshWirelessData) {
            double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD, MAX_FORWARD);
            double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
            double up = abs(controllerMessage.joystick1.u) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.u, -1, 1, -MAX_GRABBER, MAX_GRABBER);
            Serial.println("UPDATE STEPOINTSSSS");
            updateJoystickSetpoints(forward + turn, forward - turn, up);
        }
        return;
    }

    Serial.println(scenario);

    if (scenario == "POINT_TO_POINT") {
        switch (state) {
            case 0: {
                //initialbearing = atan2(robotMessage.y, robotMessage.x);
                Serial.println("Case 0 Started: Arcing towards Goal");
                double initialbearing = heading;
                double dx_goal  = abs(goalX);
                double dy_goal  = abs(goalY);
                double rho    = sqrt(dx_goal*dx_goal + dy_goal*dy_goal);
                double dtheta1 = atan2(dx_goal, dy_goal);
                //double error  = normalizeAngle(dtheta1 - abs(heading) - (PI/2));
                double error = dtheta1 - abs(heading);
                Serial.println(dtheta1);

                Serial.printf("Case 0: error=%.3f  thresh=0.005\n", error);
                if (fabs(error) > 0.02) {
                    robotVelocity = 0.2;
                    if (goalX < 0) {
                        k             = (error > 0 ? +1 : 0) * (1.0/0.25);
                        //k = 1/0.25;
                    } else {
                        k             = (error > 0 ? -1 : 0) * (1.0/0.25);
                        //k = -1/0.25;
                    }
                    Serial.print(k);
                    
                    } else {
                    state++;
                    robotMessage.x = 0;
                    robotMessage.y = 0;
                }
                    break;
            }
            case 1:{
                Serial.println("Case 1 Started: Driving towards Goal");
                double dx_goal2  = abs(goalX) - abs(robotMessage.x);
                double dy_goal2  = abs(goalY) - abs(robotMessage.y);
                double distance_to_goal = sqrt(dx_goal2*dx_goal2 + dy_goal2*dy_goal2);
                double dtheta1 = atan2(dy_goal2, dx_goal2);
                Serial.printf("Case 1: dist=%.3f\n", distance_to_goal);
                Serial.println(robotMessage.x);
                Serial.println(robotMessage.y);
                if (distance_to_goal > 0.1) {
                //if (robotMessage.x<abs(goalX) && robotMessage.y<abs(goalY)){
                //if (dx_goal2)
                    Serial.println("True");
                    robotVelocity = 0.2;
                    k = 0;
                } else {
                    Serial.println("False");
                    state++;
                }
                break;
            }
            case 2:{
                Serial.println("Case 2 Started: Aligning towards towards Goal");
                double error = finalheading - heading;
                Serial.printf("Case 2: error=%.3f\n", error);
                if (fabs(error) > 0.05) {
                    robotVelocity = 0.2;
                    if (finalheading > 0) {
                        k             = (abs(error) > 0 ? +1 : 0) * (1.0/0.25);
                        //k = 1/0.25;
                    } else {
                        k             = (abs(error) > 0 ? -1 : 0) * (1.0/0.25);
                        //k = -1/0.25;
                    }
                
                } else {
                    state++;
                    state++;
                    state++;
                    state++;
                    robotMessage.x = 0;
                    robotMessage.y = 0;
                    Serial.println("Path Done");
                }
                break;
            }
            case 3:{
                Serial.println("Case 3 Started: Final Alignment");
                double dx_align_2  = goalX - robotMessage.x;
                double dy_align_2  = goalY - robotMessage.y;
                double rho    = sqrt(dx_align_2*dx_align_2 + dy_align_2*dy_align_2);
                double dtheta1 = atan2(dy_align_2, dx_align_2);
                double dist = sqrt(dx_align_2*dx_align_2 + dy_align_2*dy_align_2);
                if (robotMessage.x>goalX && robotMessage.y>goalY) {
                    // we backed past the goal → reverse
                    robotVelocity = -0.2;
                    k             = 0;
                }
                else if (robotMessage.x<goalX && robotMessage.y<goalY) {
                    // still before the goal → drive forward
                    robotVelocity = 0.2;
                    k             = 0;
                }
                else {
                    state++;
                    robotMessage.x = 0;
                    robotMessage.y = 0;
                }
                break;
            }
            default: {
                // If none of the states, robot should just stop
                robotVelocity = 0;
                k = 0;
                break;
            }
        }
    } else if (scenario == "TURNLEFT") {
        switch (state) {
            case 0: {
                //initialbearing = atan2(robotMessage.y, robotMessage.x);
                Serial.println("Case 0 Started: Arching towards Goal");
                double initialbearing = heading;
            
                //double error  = normalizeAngle(dtheta1 - abs(heading) - (PI/2));
                double error = desiredheading - heading;
                Serial.println(desiredheading);
                Serial.println(heading);

                Serial.printf("Case 0: error=%.3f  thresh=0.005\n", error);
                if (fabs(error) > 0.02) {
                    robotVelocity = 0.3;
                  
                    k             = (abs(error) > 0 ? +1 : 0) * (1.0/0.25);
                    
                    Serial.print(k);
                    
                    } else {
                    state++;
                    robotMessage.x = 0;
                    robotMessage.y = 0;
                    Serial.println("Path Done");
                }
    
                    break;
            }
            
            default: {
                // If none of the states, robot should just stop
                robotVelocity = 0;
                k = 0;
                break;
            }
        }
    } else if (scenario == "REVERSE") {
       switch (state) {
            case 0: {
                Serial.println("Case 0 Started: Driving towards Goal");
                double dx_goal2  = abs(goalX) - abs(robotMessage.x);
                double dy_goal2  = abs(goalY) - abs(robotMessage.y);
                double distance_to_goal = sqrt(dx_goal2*dx_goal2 + dy_goal2*dy_goal2);
                double dtheta1 = atan2(dy_goal2, dx_goal2);
                Serial.printf("Case 1: dist=%.3f\n", distance_to_goal);
                Serial.println(robotMessage.x);
                Serial.println(robotMessage.y);
                //if (distance_to_goal > 0.1) {
                if (abs(robotMessage.x)<abs(goalX) || abs(robotMessage.y)<abs(goalY)){
                //if (dx_goal2)
                    Serial.println("True");
                    robotVelocity = -0.2;
                    k = 0;
                } else {
                    Serial.println("Path Done");
                    state++;
                }
                break;
            }
            
            default: {
                // If none of the states, robot should just stop
                robotVelocity = 0;
                k = 0;
                break;
            }
        } 
    }

    setWheelVelocities(robotVelocity, k);
    
    
}

void updateOdometry(double heading)
{
  // take angles from traction wheels only since they don't slip
  currPhiL = encoders[2].getPosition();
  currPhiR = -encoders[3].getPosition();

  double dPhiL = currPhiL - prevPhiL;
  double dPhiR = currPhiR - prevPhiR;
  prevPhiL = currPhiL;
  prevPhiR = currPhiR;

  float dtheta = rr / (2 * bb) * (dPhiR - dPhiL);
  float dx = rr / 2.0 * (cos(heading) * dPhiR + cos(heading) * dPhiL);
  float dy = rr / 2.0 * (sin(heading) * dPhiR + sin(heading) * dPhiL);

  // Update robot message
  robotMessage.millis = millis();
  robotMessage.x += dy;
  robotMessage.y += dx;
  robotMessage.theta += dtheta;
}


