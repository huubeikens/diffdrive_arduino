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
  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return serial_conn_.isOpen(); }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);


private:
  serial::Serial serial_conn_;  ///< Underlying serial connection 
  rclcpp::Logger logger_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H