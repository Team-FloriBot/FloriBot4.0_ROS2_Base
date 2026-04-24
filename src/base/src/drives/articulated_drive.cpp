#include "drives/articulated_drive.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <base/msg/wheels.hpp>
#include <stdexcept>

// Konstruktoren
// ----------------------
ArticulatedDrive::ArticulatedDrive()
{
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}
ArticulatedDrive::ArticulatedDrive(double axesLength, double wheelDiameter, double frontLength, double rearLength)
{
    setParam(axesLength, wheelDiameter, frontLength, rearLength);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
}


ArticulatedDrive::~ArticulatedDrive() {}

// Inverse Kinematic
// ----------------------
articulatedWheelSpeed ArticulatedDrive::inverseKinematics(geometry_msgs::msg::Twist cmdVelMsg, double angle)
{
    articulatedWheelSpeed retVal;

    // calculate inverse kinematic by hand
    targetSpeed_ = cmdVelMsg.linear.x;
    targetOmega_ = cmdVelMsg.angular.z;

    if (targetSpeed_ >= 0)
    {
        // winkel muss invertiert werden
        angle = -angle;
        retVal.Front.leftWheel  = (1.0/wheelRadius_) * targetSpeed_ - (axesLength_/(2*wheelRadius_))* targetOmega_;
        retVal.Front.rightWheel = (1.0/wheelRadius_) * targetSpeed_ + (axesLength_/(2*wheelRadius_))* targetOmega_;
        retVal.Rear.leftWheel = (cos(angle)/wheelRadius_ - (axesLength_ * sin(angle))/(wheelBase_ * wheelRadius_)) * targetSpeed_ + ((wheelBase_*sin(angle) / (2*wheelRadius_))+ (axesLength_ * cos(angle))/(2*wheelRadius_)) * targetOmega_;
        retVal.Rear.rightWheel= (cos(angle)/wheelRadius_ + (axesLength_ * sin(angle))/(wheelBase_ * wheelRadius_)) * targetSpeed_ + ((wheelBase_*sin(angle) / (2*wheelRadius_))- (axesLength_ * cos(angle))/(2*wheelRadius_)) * targetOmega_;
        
    }

    else
    {
        targetOmega_ = -targetOmega_;
        retVal.Front.leftWheel = (cos(angle)/wheelRadius_ + (axesLength_ * sin(angle))/(wheelBase_ * wheelRadius_)) * targetSpeed_ + ((wheelBase_*sin(angle) / (2*wheelRadius_))- (axesLength_ * cos(angle))/(2*wheelRadius_)) * targetOmega_;
        retVal.Front.rightWheel= (cos(angle)/wheelRadius_ - (axesLength_ * sin(angle))/(wheelBase_ * wheelRadius_)) * targetSpeed_ + ((wheelBase_*sin(angle) / (2*wheelRadius_))+ (axesLength_ * cos(angle))/(2*wheelRadius_)) * targetOmega_;
        retVal.Rear.leftWheel =  (1.0/wheelRadius_) * targetSpeed_ + (axesLength_/(2*wheelRadius_))* targetOmega_;
        retVal.Rear.rightWheel = (1.0/wheelRadius_) * targetSpeed_ - (axesLength_/(2*wheelRadius_))* targetOmega_;
    }


    return retVal;
}

// Forward Kinematic
// ----------------------
geometry_msgs::msg::Pose2D ArticulatedDrive::forwardKinematics(articulatedWheelSpeed WheelSpeed, rclcpp::Time Timestamp)
{
        double deltaTime = (Timestamp - TimeStamp_).seconds();
        TimeStamp_ = Timestamp;

        WheelSpeed_ = WheelSpeed.Front;
        Speed_.linear.x = (WheelSpeed_.leftWheel * wheelRadius_ + WheelSpeed_.rightWheel * wheelRadius_) / 2.0;
        Speed_.angular.z = (WheelSpeed_.rightWheel * wheelRadius_ - WheelSpeed_.leftWheel * wheelRadius_) / axesLength_;

        Pose_.x += Speed_.linear.x * deltaTime * cos(Pose_.theta + 0.5 * Speed_.angular.z * deltaTime);
        Pose_.y += Speed_.linear.x * deltaTime * sin(Pose_.theta + 0.5 * Speed_.angular.z * deltaTime);
        Pose_.theta += Speed_.angular.z * deltaTime;

        return Pose_;
}

// Parameter setzen
// ----------------------
void ArticulatedDrive::setParam(double AxesLength, double WheelDiameter, double frontLength, double rearLength)
{
        reset();
        axesLength_ = AxesLength;
        wheelDiameter_ = WheelDiameter;
        wheelRadius_ = wheelDiameter_ / 2.0;
        wheelCircumference_ = 2.0 * M_PI * wheelDiameter_ / 2.0;
        frontLength_ = frontLength;
        rearLength_ = rearLength;
        wheelBase_ = frontLength + rearLength;
}

void ArticulatedDrive::reset()
{
    Pose_.theta = 0.0;
    Pose_.x = 0.0;
    Pose_.y = 0.0;

    WheelSpeed_.leftWheel = 0.0;
    WheelSpeed_.rightWheel = 0.0;

    rclcpp::Clock clock(RCL_SYSTEM_TIME);
    TimeStamp_ = clock.now();
}

// Aktuelle Position vom Vorderwagen zurückgeben
// ----------------------
geometry_msgs::msg::Pose2D ArticulatedDrive::getActualPose()
{
    return Pose_;
}

// Aktuelle Geschwindigkeit zurückgeben
// ----------------------
geometry_msgs::msg::Twist ArticulatedDrive::getSpeed()
{
    return Speed_;
}
