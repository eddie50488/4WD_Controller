#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "omni_drive_arduino/omni_drive_arduino.hpp"

namespace omni_drive_arduino // same as folder name
{
  hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo &info)
  {
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg_.first_wheel_name = info_.hardware_parameters["first_wheel_name"];
    cfg_.second_wheel_name = info_.hardware_parameters["second_wheel_name"];
    cfg_.third_wheel_name = info_.hardware_parameters["third_wheel_name"];
    cfg_.fourth_wheel_name = info_.hardware_parameters["fourth_wheel_name"];
    cfg_.device = info_.hardware_parameters["device"];
    cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
    cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

    wheel_1_.setup(cfg_.first_wheel_name);
    wheel_2_.setup(cfg_.second_wheel_name);
    wheel_3_.setup(cfg_.third_wheel_name);
    wheel_4_.setup(cfg_.fourth_wheel_name);

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      // OmniBotSystem has one state interface on each joint
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveArduinoHardware"),
            "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
            joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveArduinoHardware"),
            "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
            joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      // OmniBotSystem has one command interface on each joint
      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveArduinoHardware"),
            "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
            joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
            rclcpp::get_logger("OmniDriveArduinoHardware"),
            "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
            joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> OmniDriveArduinoHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_4_.vel));
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> OmniDriveArduinoHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_1_.name, hardware_interface::HW_IF_VELOCITY, &wheel_1_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_2_.name, hardware_interface::HW_IF_VELOCITY, &wheel_2_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_3_.name, hardware_interface::HW_IF_VELOCITY, &wheel_3_.cmd));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_4_.name, hardware_interface::HW_IF_VELOCITY, &wheel_4_.cmd));
    return command_interfaces;
  }

  hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveArduinoHardware"), "Configuring ...please wait...");
    if (comms_.connected())
    {
      comms_.disconnect();
    }
    comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveArduinoHardware"), "Successfully configured!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn OmniDriveArduinoHardware::on_cleanup(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveArduinoHardware"), "Cleaning up ...please wait...");
    if (comms_.connected())
    {
      std::cout << "Exiting" << std::endl;
      comms_.disconnect();
    }
    RCLCPP_INFO(rclcpp::get_logger("OmniDriveArduinoHardware"), "Successfully cleaned up!");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type OmniDriveArduinoHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }
    // Do nothing
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type omni_drive_arduino ::OmniDriveArduinoHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!comms_.connected())
    {
      return hardware_interface::return_type::ERROR;
    }

    int pwm1 = wheel_1_.cmd;
    int pwm2 = wheel_2_.cmd;
    int pwm3 = wheel_3_.cmd;
    int pwm4 = wheel_4_.cmd;
    
    comms_.set_motor_values(pwm1, pwm2, pwm3, pwm4);
    return hardware_interface::return_type::OK;
  }

} // namespace omni_drive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(omni_drive_arduino::OmniDriveArduinoHardware, hardware_interface::SystemInterface)
