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
    double wl = vel.omega;
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
