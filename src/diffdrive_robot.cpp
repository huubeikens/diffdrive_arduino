// This file is not used, don't know why...
/*

#include "rclcpp/rclcpp.hpp"
#include "diffdrive_arduino/diffdrive_arduino.h"

void timer_callback(DiffDriveArduino& robot)
{
  robot.enableLittleMonster();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("~");

  DiffDriveArduino robot();

  auto options = controller_manager::get_cm_node_options();
  options.arguments({
    "--ros-args",
    "--remap", "_target_node_name:__node:=dst_node_name",
    "--log-level", "info"});

  auto cm = std::make_shared<controller_manager::ControllerManager>(
    rclcpp::executor, "_target_node_name", "some_optional_namespace", options);

  controller_manager::ControllerManager cm(&robot);

  rclcpp::Rate loop_rate(10);

  while (ros::ok())
  {
    robot.read();
    //cm.update(robot.get_time(), robot.get_period());
    robot.write();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
}
*/