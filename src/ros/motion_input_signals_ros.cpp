//
// Created by shivesh on 9/14/19.
//

#include "ars_40x/ros/motion_input_signals_ros.hpp"

namespace ars_40x {

MotionInputSignalsROS::MotionInputSignalsROS(ARS_40x_CAN *ars_40x_can) :
    rclcpp::Node("motion_input_signals_ros"), ars_40x_can_(ars_40x_can) {
  speed_information_ = ars_40x_can_->get_speed_information();
  yaw_rate_information_ = ars_40x_can_->get_yaw_rate_information();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&MotionInputSignalsROS::odom_callback, this, std::placeholders::_1));
    // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
    //         this->odom_callback(*msg);
    //     });

}

MotionInputSignalsROS::~MotionInputSignalsROS() {
}

void MotionInputSignalsROS::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double speed = msg->twist.twist.linear.x;
    speed_information_->set_speed(std::abs(speed));
    if (speed < 0.0) {
        speed_information_->set_speed_direction(motion_input_signals::BACKWARD);
    } else if (speed > 0.0) {
        speed_information_->set_speed_direction(motion_input_signals::FORWARD);
    } else {
        speed_information_->set_speed_direction(motion_input_signals::STANDSTILL);
    }
    ars_40x_can_->send_radar_data(can_messages::SpeedInformation);

    yaw_rate_information_->set_yaw_rate(msg->twist.twist.angular.z * 180.0 / M_PI);
    ars_40x_can_->send_radar_data(can_messages::YawRateInformation);
}

}