#ifndef OMNI_DRIVE_ARDUINO_OMNI_DRIVE_ARDUINO_HPP_
#define OMNI_DRIVE_ARDUINO_OMNI_DRIVE_ARDUINO_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "omni_drive_arduino/visibility_control.h"

#include "omni_drive_arduino/arduino_comms.hpp"
#include "omni_drive_arduino/wheel.hpp"

namespace omni_drive_arduino
{
  class OmniDriveArduinoHardware : public hardware_interface::SystemInterface
  {
    struct Config
    {
      std::string first_wheel_name = "";
      std::string second_wheel_name = "";
      std::string third_wheel_name = "";
      std::string fourth_wheel_name = "";
      std::string device = "";
      int baud_rate = 0;
      int timeout_ms = 0;
    };

  public:
  
    RCLCPP_SHARED_PTR_DEFINITIONS(OmniDriveArduinoHardware);

    OMNI_DRIVE_ARDUINO_PUBLIC
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    OMNI_DRIVE_ARDUINO_PUBLIC
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    ArduinoComms comms_;
    Config cfg_;
    Wheel wheel_1_;
    Wheel wheel_2_;
    Wheel wheel_3_;
    Wheel wheel_4_;
  };

} // namespace omni_drive_arduino

#endif // OMNI_DRIVE_ARDUINO_OMNI_DRIVE_ARDUINO_HPP_