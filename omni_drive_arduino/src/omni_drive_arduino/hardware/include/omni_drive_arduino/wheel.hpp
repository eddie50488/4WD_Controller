#ifndef OMNI_DRIVE_ARDUINO_WHEEL_HPP
#define OMNI_DRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel
{
public:
  std::string name = "";
  double cmd = 0;
  double vel = 0;

  Wheel() = default;

  Wheel(const std::string &wheel_name)
  {
    setup(wheel_name);
  }

  void setup(const std::string &wheel_name)
  {
    name = wheel_name;
  }
};

#endif // OMNI_DRIVE_ARDUINO_WHEEL_HPP
