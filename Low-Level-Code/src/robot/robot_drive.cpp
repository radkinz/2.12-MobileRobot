#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"

MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };

EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2} };

PID pids[NUM_MOTORS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false}, 
                         {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };

double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};

void setupDrive(){
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

void updateSetpoints(double left, double right, double up) {
    setpoints[2] = left;
    setpoints[3] = right;
}

void updateJoystickSetpoints(double left, double right, double up) {
    setpoints[0] = up;
    setpoints[1] = up;
    setpoints[2] = left;
    setpoints[3] = right;
}

void updatePIDs() {
    // Serial.println("setpoints");
    // Serial.println(setpoints[0]);
    // Serial.println(setpoints[1]);
    // Serial.println(setpoints[2]);
    // Serial.println(setpoints[3]);

    // Serial.println("control efforts");
    // Serial.println(controlEfforts[0]);
    // Serial.println(controlEfforts[1]);
    // Serial.println(controlEfforts[2]);
    // Serial.println(controlEfforts[3]);

    //Serial.println("velocities");
    // Serial.println(velocities[0]);
    // Serial.println(velocities[1]);
    // Serial.println(velocities[2]);
    // Serial.println(velocities[3]);

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity();
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);

        if (setpoints[2] == 0 && setpoints[3] == 0) {
             controlEfforts[2] = 0;
             controlEfforts[3] = 0;
        }

        motors[i].drive(controlEfforts[i]);
        

    }
}

void resetGripper() {
    setpoints[0] = 0;
    setpoints[1] = 0; 
}

void useGripper(float grip) {
    Serial.println("use gripper");
    Serial.println(setpoints[0]);
    Serial.println(setpoints[1]);
    Serial.println(setpoints[2]);
    Serial.println(setpoints[3]);

    if (grip == 1) {
        //updateSetpoints(0, 0, -1);
        setpoints[0] = -3;
        setpoints[1] = -3; 
    } else if (grip == -1) {
        //updateSetpoints(0, 0, 1);
        setpoints[0] = 3;
        setpoints[1] = 3; 
    } else {
        setpoints[0] = 0;
        setpoints[1] = 0; 
    }
}