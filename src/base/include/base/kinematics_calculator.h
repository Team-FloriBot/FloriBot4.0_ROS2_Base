#ifndef BASE_KINEMATICS_CALCULATOR_H
#define BASE_KINEMATICS_CALCULATOR_H

namespace base {

struct ArticulatedWheelSpeedSet
{
  double fl{0.0};
  double fr{0.0};
  double rl{0.0};
  double rr{0.0};
};

class KinematicsCalculator
{
public:
  KinematicsCalculator(double track_width_m, double wheel_radius_m,
                       double front_joint_distance_m, double rear_joint_distance_m);

  ArticulatedWheelSpeedSet calculateWheelSpeeds(
    double linear_velocity_mps,
    double yaw_rate_radps,
    double articulation_angle_rad) const;

private:
  double track_width_{0.0};
  double wheel_radius_{0.0};
  double front_joint_distance_{0.0};
  double rear_joint_distance_{0.0};
};

}  // namespace base

#endif
