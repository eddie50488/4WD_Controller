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

  // RobotVelocity Kinematics::getBodyVelocity(const std::vector<double> & wheels_vel) {
  //   RobotVelocity vel;
  //   double wm1 = wheels_vel.at(0);
  //   double wm2 = wheels_vel.at(1);
  //   double wm3 = wheels_vel.at(2);
  //   double wm4 = wheels_vel.at(3);

  //   vel.vx = beta_ * (wm2 - wm3);
  //   vel.vy = alpha_ * (-wm1 + (0.5 * (wm2 + wm3)));
  //   vel.omega = ((1/robot_params_.robot_radius)*alpha_) * ((sin_gamma_ * wm1) + (0.5 * (wm2 + wm3)));

  //   vel.vx *= robot_params_.wheel_radius;
  //   vel.vy *= robot_params_.wheel_radius;
  //   vel.omega *= robot_params_.wheel_radius;

  //   return vel;
  // }

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
    // cos_gamma_ = cos(robot_params_.gamma);
    // sin_gamma_ = sin(robot_params_.gamma);
    // alpha_ = 1 / (sin_gamma_ + 1);
    // beta_ = 1 / (2*cos_gamma_);
  }

  Kinematics::~Kinematics() {}

} // namespace omnidirectional_controllers
