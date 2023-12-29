#ifndef DIFFDRIVE_ARDUINO_WHEEL_H
#define DIFFDRIVE_ARDUINO_WHEEL_H

#include <string>
#include <chrono>

class Wheel
{
    public:
    Wheel()         = default;
    void            setup(const std::string & aName, int aMsPerRevelation, double aMaxVelocity);
    std::string&    getName();
    int             getMotorValue();
    void            update();

    double          mPosition;              // Angle in radians (pi)
    double          mVelocity;              // Actual velocity in meter/second
    double          mCommandedVelocity;     // Requested velocity in meter/second

    private:
    // Wheel specific variables
    double          mMaxVelocity;           // Velocity when wheel is turning
    int             mMsPerRevelation;       // Time for 1 revelation in ms
    std::string     mName;                  // Name of the wheel

    // State variables
    unsigned long   mUpdateTimestamp;
    int             mMotorValue;            // PWM value for the motor
};


#endif // DIFFDRIVE_ARDUINO_WHEEL_H