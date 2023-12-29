#include "diffdrive_arduino/wheel.h"

#include <cmath>


void Wheel::setup(const std::string & aName, int aMsPerRevelation, double aMaxVelocity)
{
  mName               = aName;
  mMsPerRevelation    = aMsPerRevelation;
  mMaxVelocity        = aMaxVelocity;
  mVelocity           = 0;
  mPosition           = 0;
  mCommandedVelocity  = 0;
  mMotorValue         = 0;
}

void Wheel::update()
{
  if ( mVelocity == 0 && mCommandedVelocity != 0) {
    // Wheel is not turning but is commanded to turn
    mUpdateTimestamp = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1); // Starting time of wheel turning
  } 

  unsigned long lNow = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1);

  if (mCommandedVelocity > 0) {
    // Wheel is commanded to go forward
    mPosition += ( ((2*M_PI)/ mMsPerRevelation ) * (lNow - mUpdateTimestamp) ); // Add delta position
    mVelocity = mMaxVelocity;
    mMotorValue = 255;
  }
  else if (mCommandedVelocity < 0) {
    // Wheel is commanded to go backward
    mPosition -= ( ((2*M_PI)/ mMsPerRevelation ) * (lNow - mUpdateTimestamp) ); // Substract delta position
    mVelocity = -mMaxVelocity;
    mMotorValue = -255;
  } else {
    // Commanded velocity is zero
    mVelocity = 0;
    mMotorValue = 0;
  }

   mUpdateTimestamp = lNow;
}

int Wheel::getMotorValue() {
  return mMotorValue;
}
std::string& Wheel::getName() {
  return mName;
}