#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <serial/serial.h>
#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/json.h>

class ArduinoComms
{


public:

  ArduinoComms();
  void setup(const std::string &aSerialDevice, int32_t aBaudrate, int32_t aTimeout);
  //void getStatus(); // TODO
  void setMotorValues(int aLeftWheelValue, int aRightWheelValue);
  void setMasterRelay(bool turnOn);
  bool isConnected();
private:
  serial::Serial mSerialConnection; 
  rclcpp::Logger mLogger;
  
  std::string sendMessage(const std::string &aMessage);

  int   mLeftWheelValue;
  int   mRightWheelValue;
  bool  mTurnOn;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H