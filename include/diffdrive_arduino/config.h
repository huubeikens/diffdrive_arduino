#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>


struct Config
{
  std::string     leftWheelName             = "left_wheel";
  std::string     rightWheelName            = "right_wheel";
  float           loopRate                  = 30;
  std::string     serialDevice              = "/dev/ttyUSB0";
  int             serialBaudrate            = 57600;
  int             timeout                   = 1000;
  int             msPerRevelationLeftWheel  = 35000;
  int             msPerRevelationRightWheel = 35000;

};


#endif // DIFFDRIVE_ARDUINO_CONFIG_H