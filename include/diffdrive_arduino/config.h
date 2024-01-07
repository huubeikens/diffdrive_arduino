#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string     leftWheelName             = "left_wheel_joint";
  std::string     rightWheelName            = "right_wheel_joint";
  float           loopRate                  = 30;
  std::string     serialDevice              = "/dev/ttyAMA0";
  int             serialBaudrate            = 57600;
  int             timeout                   = 1000;
  int             msPerRevelationLeftWheel  = 2667;
  int             msPerRevelationRightWheel = 2667;

};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H
