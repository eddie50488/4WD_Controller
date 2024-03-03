#ifndef OMNI_CONTROLLER_TYPES_HPP_
#define OMNI_CONTROLLER_TYPES_HPP_

#define DEG2RAD(deg) (deg * M_PI / 180.0)

namespace omni_drive_controller
{

  struct RobotParams
  {
    double wheel_radius;
    double robot_radius;
    double gamma;
  };

  struct RobotVelocity
  {
    double vx;    // [m/s]
    double vy;    // [m/s]
    double omega; // [rad]
  };

  // struct RobotPose {
  //   double x;        //  [m]
  //   double y;        //  [m]
  //   double theta;    // [rad]
  // };

} // namespace omni_drive_controller

#endif // OMNI_CONTROLLER__TYPES_HPP_
