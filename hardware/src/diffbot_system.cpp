// Copyright 2021 ros2_control Development Team
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

#include "diffbot_system.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace diffdrive_arduino
{
  hardware_interface::return_type DiffDriveArduinoHardware::configure(
      const hardware_interface::HardwareInfo &info)
  {
    if (configure_default(info) != hardware_interface::return_type::OK)
    {
      return hardware_interface::return_type::ERROR;
    }

    cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
    cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
    cfg_.loop_rate = stof(info_.hardware_parameters["loop_rate"]);
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = stoi(info_.hardware_parameters["timeout_ms"]);
    cfg_.enc_counts_per_rev = stoi(info_.hardware_parameters["enc_counts_per_rev"]);
    cfg_.acceleration = stoi(info_.hardware_parameters["acceleration"]);

    wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
    wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);


    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // DiffBotSystem has exactly two states and one command interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::return_type::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::return_type::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
            joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
            hardware_interface::HW_IF_POSITION);
        return hardware_interface::return_type::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("DiffDriveArduinoHardware"),
            "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::return_type::ERROR;
      }
    }

    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
  }

  std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));
    

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
       wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
       wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));
    

    return command_interfaces;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::start()
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Starting ...please wait...");

    // connect the motors

    if(!(_motors.begin(cfg_.baud_rate, cfg_.device.c_str())))
    {
          RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Connection Failed!");     
          return hardware_interface::return_type::ERROR;
    }

    for (uint8_t i=0;i<2;i++) 
    {
      _motors.WheelMode(motor_ids[i]);
    }
    _motors.syncReadBegin(sizeof(motor_ids), sizeof(rxPacket));
    // set the motors to wheel mode


    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "System Successfully started!");

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::stop()
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Stopping ...please wait...");

    _motors.end();

    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "System successfully stopped!");

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type DiffDriveArduinoHardware::read()
  {
    
    _motors.syncReadPacketTx(motor_ids, sizeof(motor_ids), SMS_STS_PRESENT_POSITION_L, sizeof(rxPacket));		
    int pos[2] = {0,0};
    int vel[2] = {0,0};
    for(uint8_t i=0; i<sizeof(motor_ids); i++)
    {
      if(!_motors.syncReadPacketRx(motor_ids[i], rxPacket)){
        RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cound not read motor");
        return hardware_interface::return_type::ERROR;
      }
      pos[i] = _motors.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
      vel[i] = _motors.syncReadRxPacketToWrod(15);//解码两个字节 bit15为方向位,参数=0表示无方向位
			
		}
    wheel_l_.enc = pos[0];
    wheel_r_.enc = pos[1];
    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_r_.pos = wheel_r_.calc_enc_angle();
    wheel_l_.vel = double(vel[0])/652.164;
    wheel_r_.vel = double(vel[1])/652.164;
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Current speeds being read: [%d] [%d] [%f] [%f] [%d] [%d]",vel[0],vel[1],wheel_l_.vel,wheel_r_.vel,pos[0],pos[1]);
    


    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type diffdrive_arduino::DiffDriveArduinoHardware::write()
  {

    int motor_l_actual_velocity = int(wheel_l_.cmd * -652.164);
    int motor_r_actual_velocity = int(wheel_r_.cmd * 652.164);
    
    
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Current speeds being written: [%d] [%d] [%f] [%f]",motor_l_actual_velocity,motor_r_actual_velocity,wheel_l_.cmd,wheel_r_.cmd);

    _motors.WriteSpe(motor_ids[0], motor_l_actual_velocity, cfg_.acceleration);
    _motors.WriteSpe(motor_ids[1], motor_r_actual_velocity, cfg_.acceleration);
    
    return hardware_interface::return_type::OK;
  }

} // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
