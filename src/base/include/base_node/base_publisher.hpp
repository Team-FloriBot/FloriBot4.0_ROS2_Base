#ifndef KINEMATICS_PUBLISHER_H
#define KINEMATICS_PUBLISHER_H

#include "drives/articulated_drive.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <base/msg/angle.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <base/msg/wheels.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// KinematicsPublisher Node
class KinematicsPublisher : public rclcpp::Node  
{
public:
    KinematicsPublisher();
    ~KinematicsPublisher();

private:
    void getParam();
    void createPublisherSubscriber();    
    void PublishSpeed();  
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);  
    void SpeedCallback(const base::msg::Wheels::SharedPtr msg);  
    void AngleCallback(const base::msg::Angle::SharedPtr msg);

    ArticulatedDrive Drive_;
    base::msg::Wheels Speedmsg_;
    
    rclcpp::TimerBase::SharedPtr CmdVelTimer_; 
    rclcpp::Publisher<base::msg::Wheels>::SharedPtr SpeedPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr OdometryPublisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSubscriber_;
    rclcpp::Subscription<base::msg::Wheels>::SharedPtr SpeedSubscriber_;
    rclcpp::Subscription<base::msg::Angle>::SharedPtr AngleSubs_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadaster_;
    double axesLength_, wheelDiameter_, frontLength_, rearLength_;
    double angle_;
    
};

#endif
