#include "base/hardware_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

using std::placeholders::_1;

namespace base {

namespace
{
constexpr std::size_t kWheelCount = 4;
constexpr std::array<const char *, kWheelCount> kWheelLabels{{"front_left", "front_right", "rear_left", "rear_right"}};
}  // namespace

HardwareNode::HardwareNode()
: Node("hardware_node")
{
  declare_parameter("wheel_cmd_topic", wheel_cmd_topic_);
  declare_parameter("wheel_ticks_topic", wheel_ticks_topic_);
  declare_parameter("articulation_topic", articulation_topic_);
  declare_parameter("joint_state_topic", joint_state_topic_);
  declare_parameter("ticks_frame_id", ticks_frame_id_);
  declare_parameter("simulation_only", simulation_only_);
  declare_parameter("ticks_per_rev", ticks_per_rev_);
  declare_parameter("publish_rate_hz", publish_rate_hz_);

  declare_parameter("articulation_position_index", articulation_position_index_);
  declare_parameter("articulation_joint_name", articulation_joint_name_);

  declare_parameter("front_left_position_index", wheel_position_indices_[0]);
  declare_parameter("front_right_position_index", wheel_position_indices_[1]);
  declare_parameter("rear_left_position_index", wheel_position_indices_[2]);
  declare_parameter("rear_right_position_index", wheel_position_indices_[3]);

  declare_parameter("front_left_velocity_index", wheel_velocity_indices_[0]);
  declare_parameter("front_right_velocity_index", wheel_velocity_indices_[1]);
  declare_parameter("rear_left_velocity_index", wheel_velocity_indices_[2]);
  declare_parameter("rear_right_velocity_index", wheel_velocity_indices_[3]);

  declare_parameter("front_left_joint_name", wheel_joint_names_[0]);
  declare_parameter("front_right_joint_name", wheel_joint_names_[1]);
  declare_parameter("rear_left_joint_name", wheel_joint_names_[2]);
  declare_parameter("rear_right_joint_name", wheel_joint_names_[3]);

  wheel_cmd_topic_ = get_parameter("wheel_cmd_topic").as_string();
  wheel_ticks_topic_ = get_parameter("wheel_ticks_topic").as_string();
  articulation_topic_ = get_parameter("articulation_topic").as_string();
  joint_state_topic_ = get_parameter("joint_state_topic").as_string();
  ticks_frame_id_ = get_parameter("ticks_frame_id").as_string();
  simulation_only_ = get_parameter("simulation_only").as_bool();
  ticks_per_rev_ = get_parameter("ticks_per_rev").as_double();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();

  articulation_position_index_ = get_parameter("articulation_position_index").as_int();
  articulation_joint_name_ = get_parameter("articulation_joint_name").as_string();

  wheel_position_indices_[0] = get_parameter("front_left_position_index").as_int();
  wheel_position_indices_[1] = get_parameter("front_right_position_index").as_int();
  wheel_position_indices_[2] = get_parameter("rear_left_position_index").as_int();
  wheel_position_indices_[3] = get_parameter("rear_right_position_index").as_int();

  wheel_velocity_indices_[0] = get_parameter("front_left_velocity_index").as_int();
  wheel_velocity_indices_[1] = get_parameter("front_right_velocity_index").as_int();
  wheel_velocity_indices_[2] = get_parameter("rear_left_velocity_index").as_int();
  wheel_velocity_indices_[3] = get_parameter("rear_right_velocity_index").as_int();

  wheel_joint_names_[0] = get_parameter("front_left_joint_name").as_string();
  wheel_joint_names_[1] = get_parameter("front_right_joint_name").as_string();
  wheel_joint_names_[2] = get_parameter("rear_left_joint_name").as_string();
  wheel_joint_names_[3] = get_parameter("rear_right_joint_name").as_string();

  if (ticks_per_rev_ <= 0.0) {
    throw std::runtime_error("ticks_per_rev must be > 0");
  }
  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0");
  }

  sub_cmd_ = create_subscription<base::msg::WheelVelocities4>(
    wheel_cmd_topic_, 10, std::bind(&HardwareNode::onWheelCmd, this, _1));

  sub_joint_state_ = create_subscription<JointState>(
    joint_state_topic_, 20, std::bind(&HardwareNode::onJointState, this, _1));

  pub_ticks_ = create_publisher<base::msg::WheelTicks4>(wheel_ticks_topic_, 20);
  pub_articulation_ = create_publisher<std_msgs::msg::Float64>(articulation_topic_, 20);

  last_update_time_ = now();
  last_joint_state_time_ = now();

  if (simulation_only_) {
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz_),
      std::bind(&HardwareNode::updateSimulation, this));
  }

  std::ostringstream mode;
  mode << (simulation_only_ ? "simulation" : "joint_state_bridge")
       << " | joint_state_topic=" << joint_state_topic_
       << " | articulation index=" << articulation_position_index_;
  RCLCPP_INFO(get_logger(), "%s", mode.str().c_str());
}

void HardwareNode::onWheelCmd(const base::msg::WheelVelocities4::SharedPtr msg)
{
  cmd_w_radps_[0] = msg->fl;
  cmd_w_radps_[1] = msg->fr;
  cmd_w_radps_[2] = msg->rl;
  cmd_w_radps_[3] = msg->rr;
}

void HardwareNode::updateSimulation()
{
  const auto stamp = now();
  const double dt = (stamp - last_update_time_).seconds();
  last_update_time_ = stamp;

  if (dt > 0.0) {
    const double ticks_per_rad = ticks_per_rev_ / (2.0 * M_PI);
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      ticks_[i] += static_cast<int64_t>(std::llround(cmd_w_radps_[i] * dt * ticks_per_rad));
    }
  }

  publishArticulation(stamp, 0.0);
  publishTicks(stamp);
}

void HardwareNode::onJointState(const JointState::SharedPtr msg)
{
  if (simulation_only_) {
    return;
  }

  const rclcpp::Time stamp =
    (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ? now() : rclcpp::Time(msg->header.stamp);

  last_joint_state_time_ = stamp;

  const int articulation_index = resolveIndex(
    *msg, articulation_position_index_, articulation_joint_name_, "articulation position");
  if (articulation_index >= 0) {
    const auto articulation = readValue(msg->position, articulation_index, "position");
    if (articulation.has_value()) {
      publishArticulation(stamp, articulation.value());
    }
  }

  bool all_positions_available = true;
  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const int idx = resolveIndex(*msg, wheel_position_indices_[i], wheel_joint_names_[i], kWheelLabels[i]);
    if (idx < 0) {
      all_positions_available = false;
      break;
    }
    const auto position_rad = readValue(msg->position, idx, "position", false);
    if (!position_rad.has_value()) {
      all_positions_available = false;
      break;
    }
    ticks_[i] = radiansToTicks(position_rad.value());
    last_joint_positions_[i] = position_rad.value();
    have_joint_positions_[i] = true;
  }

  if (!all_positions_available) {
    if (!warned_missing_position_) {
      RCLCPP_WARN(
        get_logger(),
        "JointState contains no complete wheel position vector. Falling back to wheel velocity integration.");
      warned_missing_position_ = true;
    }

    const double dt = (stamp - last_update_time_).seconds();
    if (dt > 0.0) {
      const double ticks_per_rad = ticks_per_rev_ / (2.0 * M_PI);
      bool all_velocities_available = true;
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        const int idx = resolveIndex(*msg, wheel_velocity_indices_[i], wheel_joint_names_[i], kWheelLabels[i]);
        if (idx < 0) {
          all_velocities_available = false;
          break;
        }
        const auto velocity_radps = readValue(msg->velocity, idx, "velocity", false);
        if (!velocity_radps.has_value()) {
          all_velocities_available = false;
          break;
        }
        ticks_[i] += static_cast<int64_t>(std::llround(velocity_radps.value() * dt * ticks_per_rad));
      }

      if (!all_velocities_available && !warned_missing_velocity_) {
        RCLCPP_WARN(
          get_logger(),
          "JointState contains no complete wheel velocity vector. Wheel ticks stay constant until valid data arrive.");
        warned_missing_velocity_ = true;
      }
    }
  }

  last_update_time_ = stamp;
  publishTicks(stamp);
}

void HardwareNode::publishTicks(const rclcpp::Time & stamp)
{
  base::msg::WheelTicks4 msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = ticks_frame_id_;
  msg.fl_ticks = ticks_[0];
  msg.fr_ticks = ticks_[1];
  msg.rl_ticks = ticks_[2];
  msg.rr_ticks = ticks_[3];
  pub_ticks_->publish(msg);
}

void HardwareNode::publishArticulation(const rclcpp::Time &, double articulation_angle_rad)
{
  std_msgs::msg::Float64 msg;
  msg.data = articulation_angle_rad;
  pub_articulation_->publish(msg);
}

int HardwareNode::resolveIndex(
  const JointState & msg,
  int configured_index,
  const std::string & configured_name,
  const std::string & field_name) const
{
  if (!configured_name.empty()) {
    const auto it = std::find(msg.name.begin(), msg.name.end(), configured_name);
    if (it != msg.name.end()) {
      return static_cast<int>(std::distance(msg.name.begin(), it));
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Configured %s joint name '%s' not found in joint_states. Falling back to configured index %d.",
      field_name.c_str(), configured_name.c_str(), configured_index);
  }
  return configured_index;
}

std::optional<double> HardwareNode::readValue(
  const std::vector<double> & values,
  int index,
  const std::string & field_name,
  bool warn_if_missing) const
{
  if (index < 0 || static_cast<std::size_t>(index) >= values.size()) {
    if (warn_if_missing) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "JointState %s index %d is outside valid range [0, %zu).",
        field_name.c_str(), index, values.size());
    }
    return std::nullopt;
  }
  return values[static_cast<std::size_t>(index)];
}

int64_t HardwareNode::radiansToTicks(double wheel_rotation_rad) const
{
  return static_cast<int64_t>(std::llround(wheel_rotation_rad * ticks_per_rev_ / (2.0 * M_PI)));
}

}  // namespace base

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<base::HardwareNode>());
  rclcpp::shutdown();
  return 0;
}
