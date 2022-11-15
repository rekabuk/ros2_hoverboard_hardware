// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//

#ifndef ROS2_HOVERBOARD_INTERFACE_HPP_
#define ROS2_HOVERBOARD_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

//#include "rclcpp/rclcpp.hpp"

//#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "visibility_control.h"
#include "protocol.hpp"


namespace ros2_hoverboard_hardware
{
class HoverboardJoints : public hardware_interface::SystemInterface
{
public:
 RCLCPP_SHARED_PTR_DEFINITIONS(HoverboardJoints)

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROS2_HOVERBOARD_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void protocol_recv (uint8_t c); // Function to recontruct serial packets coming from BLDC controller
  void protocol_txmt(void);

  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  //Port to connect too
  std::string port = "";
 
  // Sensor states, 0=temp 1=Battert
  std::vector<double> hw_sensor_states_;

  // Store the commands for the simulated robot
  //std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  //std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  //std::vector<double> hw_states_accelerations_;

  int port_fd;
  unsigned int msg_len = 0;
  uint8_t prev_byte = 0; // uint8_t is nice to store bytes
  uint16_t start_frame = 0;
  uint8_t* p;
  SerialFeedback msg, prev_msg;

};

}  // namespace ros2_hoverboard_hardware

#endif  // ROS2_HOVERBOARD_HARDWARE_HPP_
