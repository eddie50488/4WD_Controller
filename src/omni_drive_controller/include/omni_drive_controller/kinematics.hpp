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
    // // Forward kinematics
    // RobotVelocity getBodyVelocity(const std::vector<double> & wheels_vel);
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
