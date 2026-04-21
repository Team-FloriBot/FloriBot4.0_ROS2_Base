#ifndef ARTICULATED_DRIVE_H
#define ARTICULATED_DRIVE_H

#include <rclcpp/rclcpp.hpp>

#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

struct DifferentialWheelSpeed
{
    double leftWheel;
    double rightWheel;
};

struct articulatedWheelSpeed
{
    DifferentialWheelSpeed Front, Rear;
};


class ArticulatedDrive
{
    public:
        ArticulatedDrive();
        ArticulatedDrive(double axesLength, double wheelDiameter, double frontLength, double rearLength);
        ~ArticulatedDrive();

        articulatedWheelSpeed inverseKinematics(geometry_msgs::msg::Twist cmdVelMsg, double angle);
        geometry_msgs::msg::Pose2D forwardKinematics(articulatedWheelSpeed WheelSpeed, rclcpp::Time Timestamp);
        geometry_msgs::msg::Pose2D getActualPose();
        geometry_msgs::msg::Twist getSpeed();
        void reset();
        void setParam(double AxesLength, double WheelDiameter, double frontLength, double rearLength);
        
    private:
        std::shared_ptr<rclcpp::Clock> clock_;
        geometry_msgs::msg::Pose2D Pose_{};
        DifferentialWheelSpeed WheelSpeed_;
        geometry_msgs::msg::Twist Speed_;
        rclcpp::Time TimeStamp_;
        double axesLength_, wheelDiameter_, wheelCircumference_, wheelRadius_, frontLength_, rearLength_, wheelBase_;
        double targetSpeed_, targetOmega_;
        
};


#endif

