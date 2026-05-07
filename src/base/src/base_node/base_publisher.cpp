#include "base_node/base_publisher.hpp"

#include <base/msg/wheels.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

// Klassendefinition des Kinematics Node
KinematicsPublisher::KinematicsPublisher()
: Node("Kinematics"),
  angle_(0.0)
{
    // Parameter einlesen
    getParam();

    // Drive-Parameter setzen
    Drive_.setParam(axesLength_, wheelDiameter_, frontLength_, rearLength_);

    // Publisher und Subscriber erstellen
    createPublisherSubscriber();

    // TF-Broadcaster für odom -> base_footprint erstellen
    tf_broadaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Timer für Sollgeschwindigkeit
    CmdVelTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&KinematicsPublisher::PublishSpeed, this));
}

KinematicsPublisher::~KinematicsPublisher() {}

// Publiziert die Sollgeschwindigkeit der Räder zyklisch
void KinematicsPublisher::PublishSpeed()
{
    base::msg::Wheels tmp;

    tmp.header.stamp = this->get_clock()->now();

    tmp.front_left  = Speedmsg_.front_left;
    tmp.front_right = Speedmsg_.front_right;
    tmp.rear_left   = Speedmsg_.rear_left;
    tmp.rear_right  = Speedmsg_.rear_right;

    SpeedPublisher_->publish(tmp);

    Speedmsg_.front_left  = 0.0;
    Speedmsg_.front_right = 0.0;
    Speedmsg_.rear_left   = 0.0;
    Speedmsg_.rear_right  = 0.0;
}

// Parameter einlesen
void KinematicsPublisher::getParam()
{
    this->declare_parameter("axesLength", 0.335);
    this->declare_parameter("wheelDiameter", 0.28);
    this->declare_parameter("frontLength", 0.38);
    this->declare_parameter("rearLength", 0.38);

    this->get_parameter("axesLength", axesLength_);
    this->get_parameter("wheelDiameter", wheelDiameter_);
    this->get_parameter("frontLength", frontLength_);
    this->get_parameter("rearLength", rearLength_);
}

// Publisher und Subscriber erstellen
void KinematicsPublisher::createPublisherSubscriber()
{
    // Odometry Publisher
    OdometryPublisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/odom", 1);

    // JointState Publisher für robot_state_publisher
    JointStatePublisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    // Sollgeschwindigkeit an PLC
    SpeedPublisher_ =
        this->create_publisher<base::msg::Wheels>("engine/targetSpeed", 1);

    // cmd_vel von Nav2 / Teleop
    CmdVelSubscriber_ =
        this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel",
            1,
            std::bind(
                &KinematicsPublisher::CmdVelCallback,
                this,
                std::placeholders::_1));

    // Istgeschwindigkeit von PLC
    SpeedSubscriber_ =
        this->create_subscription<base::msg::Wheels>(
            "engine/actualSpeed",
            1,
            std::bind(
                &KinematicsPublisher::SpeedCallback,
                this,
                std::placeholders::_1));

    // Knickwinkel von PLC
    AngleSubs_ =
        this->create_subscription<base::msg::Angle>(
            "/sensors/bodyAngle",
            10,
            std::bind(
                &KinematicsPublisher::AngleCallback,
                this,
                std::placeholders::_1));
}

// Berechnet Soll-Radgeschwindigkeiten aus cmd_vel
void KinematicsPublisher::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    articulatedWheelSpeed wheelspeed;

    wheelspeed = Drive_.inverseKinematics(*msg, angle_);

    Speedmsg_.front_left  = wheelspeed.Front.leftWheel;
    Speedmsg_.front_right = wheelspeed.Front.rightWheel;
    Speedmsg_.rear_left   = wheelspeed.Rear.leftWheel;
    Speedmsg_.rear_right  = wheelspeed.Rear.rightWheel;
}

// Berechnet Odometry und TF odom -> base_footprint aus Ist-Radgeschwindigkeit
void KinematicsPublisher::SpeedCallback(const base::msg::Wheels::SharedPtr msg)
{
    articulatedWheelSpeed actual_speed;
    geometry_msgs::msg::Pose2D odom_pose;
    geometry_msgs::msg::TransformStamped transform_msg;
    nav_msgs::msg::Odometry odom_msg;
    tf2::Quaternion q;

    actual_speed.Front.leftWheel  = msg->front_left;
    actual_speed.Front.rightWheel = msg->front_right;
    actual_speed.Rear.leftWheel   = msg->rear_left;
    actual_speed.Rear.rightWheel  = msg->rear_right;

    rclcpp::Time stamp(msg->header.stamp);

    odom_pose = Drive_.forwardKinematics(actual_speed, stamp);

    q.setRPY(0.0, 0.0, odom_pose.theta);
    q.normalize();

    // TF: odom -> base_footprint
    transform_msg.header.stamp = msg->header.stamp;
    transform_msg.header.frame_id = "odom";
    transform_msg.child_frame_id = "base_footprint";

    transform_msg.transform.translation.x = odom_pose.x;
    transform_msg.transform.translation.y = odom_pose.y;
    transform_msg.transform.translation.z = 0.0;

    transform_msg.transform.rotation.x = q.x();
    transform_msg.transform.rotation.y = q.y();
    transform_msg.transform.rotation.z = q.z();
    transform_msg.transform.rotation.w = q.w();

    // Odometry
    odom_msg.header.stamp = msg->header.stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";

    odom_msg.pose.pose.position.x = odom_pose.x;
    odom_msg.pose.pose.position.y = odom_pose.y;
    odom_msg.pose.pose.position.z = 0.0;

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Geschwindigkeit im child_frame
    odom_msg.twist.twist = Drive_.getSpeed();

    OdometryPublisher_->publish(odom_msg);
    tf_broadaster_->sendTransform(transform_msg);
}

// Publiziert body_angle als JointState für robot_state_publisher
void KinematicsPublisher::AngleCallback(const base::msg::Angle::SharedPtr msg)
{
    angle_ = msg->angle;

    sensor_msgs::msg::JointState joint_state;

    // Möglichst Timestamp der Winkelmessung verwenden
    joint_state.header.stamp = msg->header.stamp;

    joint_state.name.push_back("body_angle");
    joint_state.position.push_back(angle_);

    JointStatePublisher_->publish(joint_state);
}
