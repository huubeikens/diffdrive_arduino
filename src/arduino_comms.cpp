#include "diffdrive_arduino/arduino_comms.h"
// #include <ros/console.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>

ArduinoComms::ArduinoComms()
: mLogger(rclcpp::get_logger("DiffDriveArduino"))
{  

}

void ArduinoComms::setup(const std::string &aSerialDevice, int32_t aBaudRate, int32_t aTimeout)
{  
    mSerialConnection.setPort(aSerialDevice);
    mSerialConnection.setBaudrate(aBaudRate);
    serial::Timeout tt = serial::Timeout::simpleTimeout(aTimeout);
    mSerialConnection.setTimeout(tt); // This should be inline except setTimeout takes a reference and so needs a variable
    mSerialConnection.open();
    RCLCPP_INFO_STREAM(mLogger,"setup serial connection : port=[]" << aSerialDevice << "], baudrate=[" << aBaudRate << "], timeout=[" << aTimeout << "]");
}

/* TODO
void ArduinoComms::getStatus(int &val_1, int &val_2)
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
*/

void ArduinoComms::setMasterRelay(bool turnOn)
{
    std::string message;
 
    if (turnOn) {
        message = "r on";
    } else {
        message = "r off";
    }
    sendMessage(message);

    message = "s";
     sendMessage(message);
}


void ArduinoComms::setMotorValues(int aLeftWheelValue, int aRightWheelValue)
{
    std::stringstream message;
    message << "m " << aLeftWheelValue << " " << aRightWheelValue;
    sendMessage(message.str());
}

std::string ArduinoComms::sendMessage(const std::string &aMessage)
{
    mSerialConnection.write(aMessage + "\r");
    std::string lresponse = mSerialConnection.readline();

    RCLCPP_INFO_STREAM(mLogger,"sendMessage : message=[" << aMessage << "] " << "response=[" << lresponse << "]");

    return lresponse;
}

bool ArduinoComms::isConnected() { 
    return mSerialConnection.isOpen(); 
}