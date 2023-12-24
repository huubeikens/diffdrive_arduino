#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

ArduinoComms::ArduinoComms()     : logger_(rclcpp::get_logger("DiffDriveArduino"))
{  

}

ArduinoComms::ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
      : serial_conn_(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms)), logger_(rclcpp::get_logger("DiffDriveArduino"))
{  

}

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{  
    serial_conn_.setPort(serial_device);
    serial_conn_.setBaudrate(baud_rate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(timeout_ms);
    serial_conn_.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    serial_conn_.open();
    // serial_conn_.(serial_device, baud_rate, serial::Timeout::simpleTimeout(timeout_ms));

}


void ArduinoComms::sendEmptyMsg()
{
    std::string response = sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
    std::string response = sendMsg("s\r");

    Json::Reader jsonReader;
    Json::Value  statusJsonObj;
    jsonReader.parse(response, statusJsonObj);
  
    const Json::Value& motorStatusObj = statusJsonObj["m"];
    val_1 = motorStatusObj[0].asInt();
    val_2 = motorStatusObj[1].asInt();
    
    RCLCPP_INFO_STREAM(logger_,"readEncoderValues val1: " << val_1);
    RCLCPP_INFO_STREAM(logger_,"readEncoderValues val2: " << val_2);
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
    std::stringstream ss1;
    ss1 << "r on" << "\r";
    sendMsg(ss1.str(), false);

    val_1 = val_1 * .9;
    val_2 = val_2 * .9;
    std::stringstream ss2;
    ss2 << "m " << val_1 << " " << val_2 << "\r";
    sendMsg(ss2.str(), false);

    RCLCPP_INFO_STREAM(logger_,"setMotorValues val1: " << val_1);
    RCLCPP_INFO_STREAM(logger_,"setMotorValues val2: " << val_2);

}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
    serial_conn_.write(msg_to_send);
    std::string response = serial_conn_.readline();

    if (print_output)
    {
        RCLCPP_INFO_STREAM(logger_,"Sent: " << msg_to_send);
        RCLCPP_INFO_STREAM(logger_,"Received: " << response);
    }

    return response;
}