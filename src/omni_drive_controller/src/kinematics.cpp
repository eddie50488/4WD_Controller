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

#include <cmath>
#include <memory>

#include "omni_drive_controller/kinematics.hpp"
#include "omni_drive_controller/types.hpp"

namespace omni_drive_controller
{

  Kinematics::Kinematics(RobotParams robot_params)
      : robot_params_(robot_params)
  {
    this->initializeParams();
  }

  Kinematics::Kinematics()
  {
    this->initializeParams();
  }

  std::vector<double> Kinematics::getWheelsAngularVelocities(RobotVelocity vel)
  {
    double vx = vel.vx;
    double vy = vel.vy;
    double wl = vel.omega * robot_params_.robot_radius;
    double on = 1;
    double idle = 0;

    // Logic to calculate wheel velocities based on kinematics
    if (vx > 0)
    {
      angular_vel_vec_[0] = on;
      angular_vel_vec_[1] = on;
      angular_vel_vec_[2] = idle;
      angular_vel_vec_[3] = idle;
    }

    if (vx < 0)
    {
      angular_vel_vec_[0] = -on;
      angular_vel_vec_[1] = -on;
      angular_vel_vec_[2] = idle;
      angular_vel_vec_[3] = idle;
    }

    if (vy > 0)
    {
      angular_vel_vec_[0] = idle;
      angular_vel_vec_[1] = idle;
      angular_vel_vec_[2] = on;
      angular_vel_vec_[3] = on;
    }

    if (vy < 0)
    {
      angular_vel_vec_[0] = idle;
      angular_vel_vec_[1] = idle;
      angular_vel_vec_[2] = -on;
      angular_vel_vec_[3] = -on;
    }

    if (wl > 0)
    {
      angular_vel_vec_[0] = on;
      angular_vel_vec_[1] = on;
      angular_vel_vec_[2] = on;
      angular_vel_vec_[3] = on;
    }

    if (wl < 0)
    {
      angular_vel_vec_[0] = -on;
      angular_vel_vec_[1] = -on;
      angular_vel_vec_[2] = -on;
      angular_vel_vec_[3] = -on;
    }

    return angular_vel_vec_;
  }

  void Kinematics::setRobotParams(RobotParams robot_params)
  {
    this->robot_params_ = robot_params;
    this->initializeParams();
  }

  void Kinematics::initializeParams()
  {
    angular_vel_vec_.reserve(OMNI_ROBOT_WHEELS);
    angular_vel_vec_ = {0, 0, 0, 0};
  }

  Kinematics::~Kinematics() {}

} // namespace omnidirectional_controllers
