// MIT License

// Copyright (c) 2022 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef OMNI_CONTROLLER_KINEMATICS_HPP_
#define OMNI_CONTROLLER_KINEMATICS_HPP_

#include <memory>
#include <vector>
#include "omni_drive_controller/types.hpp"

namespace omni_drive_controller
{

  constexpr double OMNI_ROBOT_WHEELS = 4;

  class Kinematics
  {
  public:
    explicit Kinematics(RobotParams robot_params);
    Kinematics();
    ~Kinematics();
    // Inverse kinematics
    std::vector<double> getWheelsAngularVelocities(RobotVelocity vel);
    void setRobotParams(RobotParams robot_params);

  private:
    void initializeParams();
    RobotParams robot_params_;
    std::vector<double> angular_vel_vec_;
    double cos_gamma_;
    double sin_gamma_;
    double alpha_;
    double beta_;
  };

} // namespace omni_drive_controller

#endif // OMNI_CONTROLLER_KINEMATICS_HPP_
