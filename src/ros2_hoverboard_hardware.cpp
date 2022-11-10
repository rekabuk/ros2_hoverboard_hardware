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
// Author: Andrew Baker (andrew@rekabuk.co.uk)
//

#include "ros2_hoverboard_hardware/ros2_hoverboard_hardware.hpp"
#include "ros2_hoverboard_hardware/config.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_hoverboard_hardware
{
hardware_interface::CallbackReturn HoverboardJoints::on_init(
  const hardware_interface::HardwareInfo & info)
{
 RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"),"ACB init");
 if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"),"ACB Got past init");

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//
//  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // HoverboardJoints has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardJoints"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY )
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardJoints"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY );
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardJoints"), "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardJoints"), "Joint '%s' have %s state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

  }

  RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"),"End of init");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
  ************************************************************************** 

  ************************************************************************** 
*/
hardware_interface::CallbackReturn HoverboardJoints::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "on_configure");

  if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Cannot open serial port to hoverboard");
    return hardware_interface::CallbackReturn::ERROR;
  }
    
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Serial port opened");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
  ************************************************************************** 

  ************************************************************************** 
*/
hardware_interface::CallbackReturn HoverboardJoints::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "on_shutdown");

  if ((port_fd = close(port_fd)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Cannot close serial port to hoverboard");
    return hardware_interface::CallbackReturn::ERROR;
  }
    
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Serial port closed");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
  ************************************************************************** 

  ************************************************************************** 
*/
std::vector<hardware_interface::StateInterface> HoverboardJoints::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
  }
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Export Vel L: %f R: %f", hw_states_velocities_[0], hw_states_velocities_[1]);

  return state_interfaces;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
std::vector<hardware_interface::CommandInterface> HoverboardJoints::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

   for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    //command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    //command_interfaces.emplace_back(hardware_interface::CommandInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
  }
  return command_interfaces;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
hardware_interface::CallbackReturn HoverboardJoints::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{

  // CONFIGURE THE UART -- connecting to the board
  // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
  // TODO : understand this shit
  struct termios options;
  tcgetattr(port_fd, &options);
  options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(port_fd, TCIFLUSH);
  tcsetattr(port_fd, TCSANOW, &options);

  // Set some default values
  for (std::size_t i = 0; i < hw_states_velocities_.size(); i++)
  {
    if (std::isnan(hw_states_positions_[i]))
    {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i]))
    {
      hw_states_velocities_[i] = 0;
    }
    // if (std::isnan(hw_states_accelerations_[i]))
    // {
    //   hw_states_accelerations_[i] = 0;
    // }

    //if (std::isnan(hw_commands_positions_[i]))
    //{
    //  hw_commands_positions_[i] = 0;
    //}
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    // if (std::isnan(hw_commands_accelerations_[i]))
    // {
    //   hw_commands_accelerations_[i] = 0;
    // }

    // control_level_[i] = integration_level_t::UNDEFINED;
  }

  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
hardware_interface::CallbackReturn HoverboardJoints::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (port_fd != -1) 
        close(port_fd);

    RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
hardware_interface::return_type HoverboardJoints::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Reading...");

  if (port_fd != -1) {
    uint8_t c;
    int i = 0, r = 0;

    while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024){
      //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Reading UART");
      protocol_recv(c);
    }            

    // if (i > 0)
    // last_read = ros::Time::now();

    if (r < 0 && errno != EAGAIN)
      RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Reading from serial %s failed: %d", PORT, r);
  }

  // RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
hardware_interface::return_type HoverboardJoints::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Writing...please wait...");

  // Write to the hardware
  protocol_txmt();

  // RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
  }


/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
void HoverboardJoints::protocol_recv (uint8_t byte) {
    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
    //RCLCPP_INFO(this->get_logger(), "Received a byte: %x",(uint8_t)byte);
    // if ((uint8_t)byte == 0xAB && (uint8_t)prev_byte == 0xCD){
    //     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x", start_frame);
    //     RCLCPP_INFO(this->get_logger(), "Received Start frame: %x %x", (byte) << 8, (uint8_t)prev_byte);
    // }

    // Read the start frame
    if (start_frame == START_FRAME) {
        //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Start frame recognised");
        p = (uint8_t*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
            msg.wheelR_cnt ^
            msg.wheelL_cnt ^
            msg.batVoltage ^
            msg.boardTemp ^
            msg.cmdLed);
              
            if (msg.start == START_FRAME && msg.checksum == checksum) {
            // std_msgs::msg::Float64 f;

            // f.data = (double)msg.batVoltage/100.0;
            // voltage_pub_->publish(f);

            // f.data = (double)msg.boardTemp/10.0;
            // temp_pub_->publish(f);

            // f.data = (double)msg.speedL_meas;
            // vel_pub_[0]->publish(f);
            // f.data = (double)msg.speedR_meas;
            // vel_pub_[1]->publish(f);
            hw_states_velocities_[0]= (double)msg.speedL_meas;
            hw_states_velocities_[1]= (double)msg.speedR_meas;
            //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Vel L: %d R: %d", msg.speedL_meas, msg.speedR_meas);

              // One rotation = 90, convert to radians
            hw_states_positions_[0] = ((double)(msg.wheelL_cnt)/90.0)*2.0*M_PI;
            hw_states_positions_[1] = ((double)(msg.wheelR_cnt)/90.0)*2.0*M_PI;

            // f.data = (double)msg.cmd1;
            // cmd_pub_[0]->publish(f);
            // f.data = (double)msg.cmd2;
            // cmd_pub_[1]->publish(f);
        } else {
            RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;
}

/*
  ************************************************************************** 
  
  ************************************************************************** 
*/
void HoverboardJoints::protocol_txmt() {
    if (port_fd == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Attempt to write on closed serial");
        return;
    }
    // Calculate steering from difference of left and right //TODO : change this shiiit
    //const double speed = (hw_commands_velocities_[0] + hw_commands_velocities_[1])/2.0;
    //const double steer = (hw_commands_velocities_[0] - hw_commands_velocities_[1])*2.0;
    const double speed = hw_commands_velocities_[0];
    const double steer = hw_commands_velocities_[1];

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd, (const void*)&command, sizeof(command));
    if (rc < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Error writing to hoverboard serial port");
    }
}

}  // namespace ros2_hoverboard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_hoverboard_hardware::HoverboardJoints, hardware_interface::SystemInterface)
