#include <Arduino.h>
#include "imu.h"
#include "EulerAngles.h"
#include "util.h"

#define PRINT_DELAY 100

#define IMU_RST 14
#define IMU_CS 12
#define IMU_INT 13

IMU imu(IMU_RST, IMU_CS, IMU_INT);

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

// Select a channel on the TCA9548A
void tca_select(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setup() {
    Serial.begin(115200);
    delay(1000);  // Let serial settle

    imu.setup();
    //TOF
    Wire.begin(SDA_PIN, SCL_PIN);
    Serial.println("Selecting TCA channel...");
    tca_select(TCA_CHANNEL);

    Serial.println("Initializing VL53L0X...");
    if (!lox.begin()) {
        Serial.println(F("Failed to boot VL53L0X on selected TCA channel"));
        while (1);
    }


    Serial.println("Setup complete.");
}

void loop() {
    EVERY_N_MILLIS(PRINT_DELAY) {
        imu.update();
        printEulerDeg(imu.getEulerAngles());
        //printGyroDeg(imu.getGyroReadings());

        //TOF
        VL53L0X_RangingMeasurementData_t measure;

        // Always re-select the TCA channel before each read (important for stability)
        tca_select(TCA_CHANNEL);

        // Serial.print("Reading a measurement... ");
        lox.rangingTest(&measure, false);  // Set to true for debugging

        if (measure.RangeStatus != 4) {
            Serial.print("[TOF] Distance (mm): ");
            Serial.println(measure.RangeMilliMeter);
        } else {
            Serial.println("TOF Out of range");
        }
    }
}

