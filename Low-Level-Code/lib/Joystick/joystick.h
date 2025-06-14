#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdint.h>

#define JOYSTICK_READING_MIN -1
#define JOYSTICK_READING_MAX 1

struct JoystickReading {
    float x;
    float y;
    float u;

    void print(uint8_t nTabs = 0);
    bool operator==(const JoystickReading& other);
} ;

#define JOYSTICK_ANALOG_MIN 0
#define JOYSTICK_ANALOG_MAX 4095

class Joystick {
private:
    int _xPin;
    int _yPin;
    int _uPin;
    float _alpha;
    
    JoystickReading _rawReading;
    JoystickReading _filtReading;

public:
    Joystick(int xPin, int yPin, int uPin, float alpha = 0.5);
    void setup();
    JoystickReading read(bool debugPrint = false);
} ;

#endif // JOYSTICK_H