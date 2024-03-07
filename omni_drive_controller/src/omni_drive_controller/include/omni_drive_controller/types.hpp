#ifndef OMNI_CONTROLLER_TYPES_HPP_
#define OMNI_CONTROLLER_TYPES_HPP_

#define DEG2RAD(deg) (deg * M_PI / 180.0)

namespace omni_drive_controller
{

  struct RobotParams
  {
    double wheel_radius;
  };

  struct RobotVelocity
  {
    double vx;
    double vy;
    double omega;
  };

} // namespace omni_drive_controller

#endif // OMNI_CONTROLLER__TYPES_HPP_
