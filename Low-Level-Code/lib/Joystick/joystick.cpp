# include "joystick.h"
# include "util.h"
# include <Arduino.h>

char currentKey = '\0';  // global or class-level

void JoystickReading::print(uint8_t nTabs) {
    printTabs(nTabs); Serial.printf("x: %.3f\n", x);
    printTabs(nTabs); Serial.printf("y: %.3f\n", y);
    printTabs(nTabs); Serial.printf("u: %.3f\n", u);

} 

bool JoystickReading::operator==(const JoystickReading& other) {
    return x == other.x &&
           y == other.y &&
           u == other.u;
}

Joystick::Joystick(int xPin, int yPin, int uPin, float alpha) 
    : _xPin(xPin), _yPin(yPin), _alpha(alpha), _rawReading({0, 0, 0}), _filtReading({0, 0, 0}) { }

void Joystick::setup() {
    pinMode(_xPin, INPUT);
    pinMode(_yPin, INPUT);
}

JoystickReading Joystick::read(bool debugPrint) {
    static float sim_x = 0.0f;
    static float sim_y = 0.0f;
    const float step = 0.05f;
    static float sim_u = 0.0f;

    static unsigned long lastInputTime = 0;
    const unsigned long inputTimeout = 150;  // ms

    // Read key from serial
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == 'w' || c == 'a' || c == 's' || c == 'd' || c=='m' || c=='j') {
            currentKey = c;
            lastInputTime = millis();
        }
    }

    // Reset key after timeout
    if (millis() - lastInputTime > inputTimeout) {
        currentKey = '\0';
    }

    // Simulate analog values
    if (currentKey == 'w') sim_y += step;
    else if (currentKey == 's') sim_y -= step;
    else if (currentKey == 'a') sim_x -= step;
    else if (currentKey == 'd') sim_x += step;
    else if (currentKey == 'm') sim_u += step;
    else if (currentKey == 'j') sim_u -= step;
    else {
        // Decay toward center
        sim_x *= 0.8f;
        sim_y *= 0.8f;
        sim_u *= 0.8f;
        
    }


    // Clamp
    sim_x = constrain(sim_x, JOYSTICK_READING_MIN, JOYSTICK_READING_MAX);
    sim_y = constrain(sim_y, JOYSTICK_READING_MIN, JOYSTICK_READING_MAX);
    sim_u = constrain(sim_u, JOYSTICK_READING_MIN, JOYSTICK_READING_MAX);


    _rawReading.x = sim_x;
    _rawReading.y = sim_y;
    _rawReading.u = sim_u;

    _filtReading.x = _alpha * _rawReading.x + (1 - _alpha) * _filtReading.x;
    _filtReading.y = _alpha * _rawReading.y + (1 - _alpha) * _filtReading.y;
    _filtReading.u = _alpha * _rawReading.u + (1 - _alpha) * _filtReading.u;

    if (debugPrint && Serial) {
        Serial.printf("Simulated joystick input via key: %c\n", currentKey);
        _filtReading.print();
    }

    return _filtReading;
}
