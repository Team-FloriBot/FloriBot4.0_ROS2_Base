#ifndef BASE_KINEMATICS_NODE_H
#define BASE_KINEMATICS_NODE_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "base/kinematics_calculator.h"
#include "base/msg/wheel_ticks4.hpp"
#include "base/msg/wheel_velocities4.hpp"

namespace base {

class KinematicsNode : public rclcpp::Node
{
public:
  KinematicsNode();

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void articulationCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void wheelTicksCallback(const base::msg::WheelTicks4::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult onParams(const std::vector<rclcpp::Parameter>& params);

  void rebuildCalculator();
  void publishOdom(const rclcpp::Time & stamp, double vx, double wz);

  static int64_t deltaModulo(int64_t cur, int64_t last, int64_t mod);
  static double normalizeAngle(double a);

  double track_width_{0.385};
  double wheel_radius_{0.1};
  double front_joint_distance_{0.55};
  double rear_joint_distance_{0.55};
  double ticks_per_rev_{131000.0};
  bool unwrap_modulo_{false};
  int modulo_ticks_{131000};
  bool publish_tf_{true};
  std::string odom_source_axle_{"front"};

  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string articulation_topic_{"/base/articulation_angle"};
  std::string wheel_cmd_topic_{"/base/wheel_commands4"};
  std::string wheel_ticks_topic_{"/base/wheel_ticks4"};
  std::string odom_topic_{"/odom"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_articulation_;
  rclcpp::Subscription<base::msg::WheelTicks4>::SharedPtr sub_ticks_;
  rclcpp::Publisher<base::msg::WheelVelocities4>::SharedPtr pub_wheel_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<KinematicsCalculator> kinematics_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_cb_;

  double articulation_angle_rad_{0.0};
  bool have_prev_ticks_{false};
  int64_t prev_fl_{0}, prev_fr_{0}, prev_rl_{0}, prev_rr_{0};
  rclcpp::Time last_odom_time_;
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
};

}  // namespace base

#endif
