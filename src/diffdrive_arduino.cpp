#include "diffdrive_arduino/diffdrive_arduino.h"


#include "hardware_interface/types/hardware_interface_type_values.hpp"




DiffDriveArduino::DiffDriveArduino() : mLogger(rclcpp::get_logger("DiffDriveArduino"))
{
  rcutils_ret_t ret = rcutils_logging_set_logger_level("DiffDriveArduino", RCUTILS_LOG_SEVERITY_INFO); 
}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(mLogger, "Configuring...");

  mConfig.leftWheelName               = info_.hardware_parameters["left_wheel_name"];
  mConfig.rightWheelName              = info_.hardware_parameters["right_wheel_name"];
  mConfig.loopRate                    = std::stof(info_.hardware_parameters["loop_rate"]);
  mConfig.serialDevice                = info_.hardware_parameters["device"];
  mConfig.serialBaudrate              = std::stoi(info_.hardware_parameters["baud_rate"]);
  mConfig.timeout                     = std::stoi(info_.hardware_parameters["timeout"]);
  mConfig.msPerRevelationLeftWheel    = std::stoi(info_.hardware_parameters["ms_per_revelation_left_wheel"]);
  mConfig.msPerRevelationRightWheel   = std::stoi(info_.hardware_parameters["ms_per_revelation_right_wheel"]);

  double lWheelRadius = 0.098; // TODO; get this value from the my_controllers.yaml
  double lMaxVelocityLeftWheel = (2*M_PI * lWheelRadius)/(mConfig.msPerRevelationLeftWheel/1000.0); // Calculate velocity in meter per seconds
  double lMaxVelocityRightWheel = (2*M_PI * lWheelRadius)/(mConfig.msPerRevelationRightWheel/1000.0); // Calculate velocity in meter per seconds

  // Set up the wheels
  mLeftWheel.setup(mConfig.leftWheelName, mConfig.msPerRevelationLeftWheel, lMaxVelocityLeftWheel);
  mRightWheel.setup(mConfig.rightWheelName, mConfig.msPerRevelationRightWheel, lMaxVelocityRightWheel);

  // Set up the Arduino
  mArduino.setup(mConfig.serialDevice, mConfig.serialBaudrate, mConfig.timeout);
  // TODO: remove this sleep!!! We also need to wait 2 seconds for the serial interface on the arduino to settle.
  std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //
  enableLittleMonster();

  RCLCPP_INFO(mLogger, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  RCLCPP_INFO(mLogger, "Setup publishing position and velocity for each wheel");
  state_interfaces.emplace_back(hardware_interface::StateInterface(mLeftWheel.getName(), hardware_interface::HW_IF_VELOCITY, &mLeftWheel.mVelocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(mLeftWheel.getName(), hardware_interface::HW_IF_POSITION, &mLeftWheel.mPosition));
  state_interfaces.emplace_back(hardware_interface::StateInterface(mRightWheel.getName(), hardware_interface::HW_IF_VELOCITY, &mRightWheel.mVelocity));
  state_interfaces.emplace_back(hardware_interface::StateInterface(mRightWheel.getName(), hardware_interface::HW_IF_POSITION, &mRightWheel.mPosition));
 
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  RCLCPP_INFO(mLogger, "Setup subscription for velocity command requests");
  command_interfaces.emplace_back(hardware_interface::CommandInterface(mLeftWheel.getName(), hardware_interface::HW_IF_VELOCITY, &mLeftWheel.mCommandedVelocity));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(mRightWheel.getName(), hardware_interface::HW_IF_VELOCITY, &mRightWheel.mCommandedVelocity));
  
  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(mLogger, "Starting Controller...");

  status_ = hardware_interface::status::STARTED;
  return return_type::OK;
}

void DiffDriveArduino::enableLittleMonster()
{
  if (!mArduino.isConnected())
  {
    RCLCPP_ERROR(mLogger, "Not connected to Arduino...");
  } else {
    RCLCPP_INFO(mLogger, "Turning Little Monster Master Relay on. It's now armed and ready!");
    mArduino.setMasterRelay(true);  
  }
}
return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(mLogger, "Stopping Controller...");
  status_ = hardware_interface::status::STOPPED;
  return return_type::OK;
}

/*
 * Callback function: Publish new wheel position and velocity states
 */ 
hardware_interface::return_type DiffDriveArduino::read()
{
  if (!mArduino.isConnected())
  {
    RCLCPP_ERROR(mLogger, "Not connected to Arduino...");
    return return_type::ERROR;
  }

  /*
   * Update wheel position and velocity topic variables (part of Wheel class)
   */
  mLeftWheel.update();
  mRightWheel.update();

  RCLCPP_DEBUG_STREAM(mLogger,"Publish left wheel  : velocity=" << mLeftWheel.mVelocity << " , position=" << mLeftWheel.mPosition);
  RCLCPP_DEBUG_STREAM(mLogger,"Publish right wheel : velocity=" << mRightWheel.mVelocity << " , position=" << mRightWheel.mPosition);
  
  return return_type::OK;

  
}

hardware_interface::return_type DiffDriveArduino::write()
{
  // Callback function when receiving a new velocity command requests
  if (!mArduino.isConnected())
  {
    RCLCPP_ERROR(mLogger, "Not connected to Arduino...");
    return return_type::ERROR;
  }

  RCLCPP_DEBUG_STREAM(mLogger,"Received left wheel velocity command  =" << mLeftWheel.mCommandedVelocity );
  RCLCPP_DEBUG_STREAM(mLogger,"Received right wheel velocity command =" << mRightWheel.mCommandedVelocity );

  mArduino.setMotorValues(mLeftWheel.getMotorValue(), mRightWheel.getMotorValue());
  return return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)