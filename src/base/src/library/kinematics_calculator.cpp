#include "base/kinematics_calculator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace base {

KinematicsCalculator::KinematicsCalculator(
  double track_width_m,
  double wheel_radius_m,
  double front_joint_distance_m,
  double rear_joint_distance_m)
: track_width_(track_width_m),
  wheel_radius_(wheel_radius_m),
  front_joint_distance_(front_joint_distance_m),
  rear_joint_distance_(rear_joint_distance_m)
{
  if (track_width_ <= 0.0) {
    throw std::runtime_error("track_width must be > 0");
  }
  if (wheel_radius_ <= 0.0) {
    throw std::runtime_error("wheel_radius must be > 0");
  }
  if (front_joint_distance_ <= 0.0 || rear_joint_distance_ <= 0.0) {
    throw std::runtime_error("front_joint_distance and rear_joint_distance must be > 0");
  }
}

ArticulatedWheelSpeedSet KinematicsCalculator::calculateWheelSpeeds(
  double linear_velocity_mps,
  double yaw_rate_radps,
  double articulation_angle_rad) const
{
  ArticulatedWheelSpeedSet out;

  const double half_track = 0.5 * track_width_;

  const double v_front = linear_velocity_mps;
  const double w_front = yaw_rate_radps;

  const double v_joint_front_x = v_front;
  const double v_joint_front_y = front_joint_distance_ * w_front;

  const double c = std::cos(articulation_angle_rad);
  const double s = std::sin(articulation_angle_rad);

  const double v_joint_rear_x =  c * v_joint_front_x + s * v_joint_front_y;
  const double v_joint_rear_y = -s * v_joint_front_x + c * v_joint_front_y;

  const double v_rear = v_joint_rear_x;
  const double w_rear = -v_joint_rear_y / rear_joint_distance_;

  out.fl = (v_front - w_front * half_track) / wheel_radius_;
  out.fr = (v_front + w_front * half_track) / wheel_radius_;
  out.rl = (v_rear  - w_rear  * half_track) / wheel_radius_;
  out.rr = (v_rear  + w_rear  * half_track) / wheel_radius_;

  return out;
}

}  // namespace base
