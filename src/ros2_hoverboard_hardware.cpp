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

  low_wrap = ENCODER_LOW_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  high_wrap = ENCODER_HIGH_WRAP_FACTOR*(ENCODER_MAX - ENCODER_MIN) + ENCODER_MIN;
  last_wheelcountR = last_wheelcountL = 0;
  multR = multL = 0;
  last_read=0.0;

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  port = info_.hardware_parameters["hoverboard_port"];
  RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"),"port <%s>", port.c_str());

  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_sensor_states_.resize( info_.sensors.size(), std::numeric_limits<double>::quiet_NaN());
 
 for (const hardware_interface::ComponentInfo & sensors : info_.sensors)
  {
    // HoverboardJoints has state interface per sensor
    if (sensors.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardSensors"),
        "Sensors '%s' has %zu state interfaces found. 1 expected.", sensors.name.c_str(),
        sensors.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

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

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("HoverboardJoints"), "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
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

  if ((port_fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Cannot open serial port to hoverboard %s", port.c_str());
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
    //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    //state_interfaces.emplace_back(hardware_interface::StateInterface(
    //  info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
  }
  //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Export Vel L: %f R: %f", hw_states_velocities_[0], hw_states_velocities_[1]);


  for (std::size_t i = 0; i < info_.sensors.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[i].name, info_.sensors[i].state_interfaces[0].name, &hw_sensor_states_[i]));
  }

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
  }

  for (std::size_t i = 0; i < hw_commands_velocities_.size(); i++)
  {

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

    if (r < 0 && errno != EAGAIN)
    {
      RCLCPP_ERROR(rclcpp::get_logger("HoverboardJoints"), "Reading from serial %s failed: %d", port.c_str(), r);
    }

    // If nothing heard for a long time then assume controller has shut down
    if ((rclcpp::Clock{}.now().seconds() - last_read) > 1.0)
    {
      hw_sensor_states_[2] = 0.0;
    }
    else
    {
      hw_sensor_states_[2] = 1.0;
    }

    if (i > 0)
    {
      last_read = rclcpp::Clock{}.now().seconds();
    }
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
        //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Got msg from hb");

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
          hw_sensor_states_[0]=(double)msg.batVoltage/100.0;

          // f.data = (double)msg.boardTemp/10.0;
          hw_sensor_states_[1]=(double)msg.boardTemp/10.0;

          // Convert RPM to RAD/S
          hw_states_velocities_[0]= (double)msg.speedL_meas * 0.10472;
          hw_states_velocities_[1]= (double)msg.speedR_meas * -0.10472;
          //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Vel L: %d R: %d", msg.speedL_meas, msg.speedR_meas);

          // Process encoder values and update odometry
          on_encoder_update(msg.wheelR_cnt, msg.wheelL_cnt);
          //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Pos L: %0.3f R: %0.3f", hw_states_positions_[0], hw_states_positions_[1]);
          //RCLCPP_INFO(rclcpp::get_logger("HoverboardJoints"), "Vel L: %0.3f R: %0.3f", hw_states_velocities_[0], hw_states_velocities_[1]);
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
    const double speed = hw_commands_velocities_[0] * 8.0;
    const double steer = hw_commands_velocities_[1] * 8.0;

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

void HoverboardJoints::on_encoder_update (int16_t right, int16_t left) 
{
    double posL = 0.0, posR = 0.0;

    // Calculate wheel position in ticks, factoring in encoder wraps
    if (right < low_wrap && last_wheelcountR > high_wrap)
        multR++;
    else if (right > high_wrap && last_wheelcountR < low_wrap)
        multR--;
    posR = right + multR*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountR = right;

    if (left < low_wrap && last_wheelcountL > high_wrap)
        multL++;
    else if (left > high_wrap && last_wheelcountL < low_wrap)
        multL--;
    posL = left + multL*(ENCODER_MAX-ENCODER_MIN);
    last_wheelcountL = left;

    // When the board shuts down and restarts, wheel ticks are reset to zero so the robot can be suddently lost
    // This section accumulates ticks even if board shuts down and is restarted   
    static double lastPosL = 0.0, lastPosR = 0.0;
    static double lastPubPosL = 0.0, lastPubPosR = 0.0;
    static bool nodeStartFlag = true;
    
    //IF there has been a pause in receiving data AND the new number of ticks is close to zero, indicates a board restard
    //(the board seems to often report 1-3 ticks on startup instead of zero)
    //reset the last read ticks to the startup values
    if ( ((rclcpp::Clock{}.now().seconds() - last_read) > 0.2) && (abs(posL) < 5) && (abs(posR) < 5)) 
    {
        lastPosL = posL;
        lastPosR = posR;
	  }

    double posLDiff = 0;
    double posRDiff = 0;

      //if node is just starting keep odom at zeros
    if (nodeStartFlag)
    {
        nodeStartFlag = false;
    }
    else
    {
        posLDiff = posL - lastPosL;
        posRDiff = posR - lastPosR;
    }

    lastPubPosL += posLDiff;
    lastPubPosR += posRDiff;
    lastPosL = posL;
    lastPosR = posR;
    
    // Convert position in accumulated ticks to position in radians
    hw_states_positions_[0] = 2.0 * M_PI * lastPubPosL/(double)TICKS_PER_ROTATION;
    hw_states_positions_[1] = 2.0 * M_PI * lastPubPosR/(double)TICKS_PER_ROTATION;

}

}  // namespace ros2_hoverboard_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_hoverboard_hardware::HoverboardJoints, hardware_interface::SystemInterface)
