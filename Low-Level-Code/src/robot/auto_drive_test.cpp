#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"
#include "imu.h"
#include "EulerAngles.h"
#include "robot_pinout.h"

//SETUP TOF
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// TCA9548A I2C multiplexer address (default)
#define TCA_ADDRESS 0x70

// Define which channel you're using on the TCA (0–7)
#define TCA_CHANNEL 0

// ESP32 I2C pins (adjust if needed)
#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
extern ControllerMessage controllerMessage;


// Select a channel on the TCA9548A
void tca_select(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


IMU imu(BNO08X_RESET, BNO08X_CS, BNO08X_INT);

// ———————————————— GOAL POSE ————————————————————————————————————————
double goalX = 0.0;            // set your desired target x (m)
double goalY = 0.0;            // set your desired target y (m)
double goalX_input = 0.0;
double goalY_input = 0.0;
double goalTheta = 0.0; // set your desired target heading (rad)
double goalTheta_input = 0.0; // set your desired target heading (rad)
double grip_input = 0.0; // set your desired target heading (rad)
double desiredheading = -0.34;
String scenario_input = "";
double theta_input = 0.0;

String scenario = "POINT_TO_POINT";
float grip = 0;

bool resetState = true;

bool isValidData(String input) {
  // Check if both "goalX:" and "goalY:" are present in the input
  if (input.indexOf("goalX:") != -1 && input.indexOf("goalY:") != -1) {
    return true;
  }

  if (input.indexOf("grip:") != -1) {
    return true;
  }


  if (input.indexOf("theta:") != -1) {
    return true;
  }

  if (input.indexOf("scenario:") != -1) {
    return true;
  }

  return false;
}

void parseData(String input) {
  Serial.println("input");
  Serial.println(input);

  // Find the position of "goalX:" and "goalY:"
  int goalXIndex = input.indexOf("goalX:");
  int goalYIndex = input.indexOf("goalY:");
  int goalThetaIndex = input.indexOf("finalHeading:");
  int scenarioIndex = input.indexOf("scenario:");
  int thetaIndex = input.indexOf("theta:");
  int gripIndex = input.indexOf("grip:");

  if (goalXIndex != -1 && goalYIndex != -1 && goalThetaIndex != -1) {
    // Extract the value after "goalX:"
    goalX_input = input.substring(goalXIndex + 6, input.indexOf(" ", goalXIndex)).toFloat();
    
    Serial.println("GOAL X");
    Serial.println(goalX_input);
    
    // Extract the value after "goalY:"
    goalY_input = input.substring(goalYIndex + 6, input.indexOf(" ", goalYIndex)).toFloat();

    // Extract the value after "goalHeading:"
    goalTheta_input = input.substring(goalThetaIndex + 13, input.length()).toFloat();

    if (goalX != goalX_input || goalY != goalY_input) {
      resetState = true;
      goalX = goalX_input;
      goalY = goalY_input;
      goalTheta = goalTheta_input;
      Serial.print("reset State");
    }
  }

  if (gripIndex != -1) {
    grip_input = input.substring(gripIndex + 5, input.length()).toFloat();


    if (grip_input != grip) {
      grip = grip_input; 
    }
  }

  if (scenarioIndex != -1) {
    scenario_input = input.substring(scenarioIndex + 9, input.indexOf(" ", scenarioIndex));
    Serial.println("SCENARIO INPUT");
    Serial.println(scenario_input);
    Serial.println(scenarioIndex);
    if (scenario == "POINT_TO_POINT" || scenario == "JOYSTICK" || scenario == "REVERSE" || scenario == "TURNLEFT" || scenario == "TURNRIGHT") {
      scenario = scenario_input;
      resetState = true;
      Serial.println("RESET SCENARIO");
    }
  }

  if (thetaIndex != -1) {
    theta_input = input.substring(thetaIndex + 6, input.indexOf(" ", thetaIndex)).toFloat();
    Serial.println("THETA INPUT");
    Serial.println(thetaIndex);
    Serial.println(theta_input);
    if (theta_input != desiredheading) {
      desiredheading = theta_input;
    }
  }
}

void setup() {
    Serial.begin(115200);
    delay(1000);  // Let serial settle
    setupDrive();
    setupWireless();
    imu.setup();

    //TOF
    // Wire.begin(SDA_PIN, SCL_PIN);
    // Serial.println("Selecting TCA channel...");
    // tca_select(TCA_CHANNEL);

    // Serial.println("Initializing VL53L0X...");
    // if (!lox.begin()) {
    //     Serial.println(F("Failed to boot VL53L0X on selected TCA channel"));
    //     while (1);
    // }
}

void loop() {
    // Check if data is available in the serial buffer
    if (Serial.available() > 0) {
        // Read the incoming data into a string
        String input = Serial.readStringUntil('\n');  // Read until a newline character
        
        // Parse the string to extract goalX and goalY if present
        if (isValidData(input)) {
        parseData(input);

        } else {
        // If the data doesn't contain goalX and goalY, ignore it
        Serial.println("Invalid data, ignoring...");
        }
    }

    // Update velocity setpoints based on trajectory at some Hz
    EVERY_N_MILLIS(20) {

        imu.update();
        EulerAngles angle = imu.getEulerAngles();
        printEulerDeg(angle);

        Serial.println("JOYSTIKKKKK");
        if (freshWirelessData) {
          if (abs(controllerMessage.joystick1.y) > 0.1) {
            Serial.println("RESET JOYATICK");
            Serial.println(controllerMessage.joystick1.y);
            scenario = "JOYSTICK";
          }
        }
        

        //TOF
        // VL53L0X_RangingMeasurementData_t measure;

        // // Always re-select the TCA channel before each read (important for stability)
        // tca_select(TCA_CHANNEL);

        // // Serial.print("Reading a measurement... ");
        // lox.rangingTest(&measure, false);  // Set to true for debugging

        // if (measure.RangeStatus != 4) {
        //     wSerial.print("[TOF] Distance (mm): ");
        //     Serial.println(measure.RangeMilliMeter);
        // } else {
        //     Serial.println("TOF Out of range");
        // }

        Serial.printf("GoalX=%.2f  GoalY=%.2f GoalTheta=%.2f desiredheading=%.2f\n", goalX, goalY, goalTheta, desiredheading);
        Serial.println("SCENARIO " + scenario);

        followTrajectory(goalX, goalY, angle.yaw, goalTheta, desiredheading, scenario, resetState); //send heading from imu

        updateOdometry(angle.yaw);

        if (scenario != "JOYSTICK") {
          useGripper(grip);
        }

        if (resetState) {
          resetState = false;
          Serial.println("Reset state");
        }
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(4000) {
    //   resetGripper();
    //   resetGrip = false;
      

    //   Serial.println("SWAP");
    //     //updateOdometry();
      sendRobotData();

    //     // Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
    //     //             robotMessage.x, robotMessage.y, robotMessage.theta);
    }  
}
