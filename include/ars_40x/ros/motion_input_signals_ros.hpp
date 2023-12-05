//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40x_MOTION_INPUT_SIGNALS_ROS_HPP
#define ARS_40x_MOTION_INPUT_SIGNALS_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>

#include "ars_40x/ars_40x_can.hpp"

namespace ars_40x {
class MotionInputSignalsROS : public rclcpp::Node {
 public:
  explicit MotionInputSignalsROS(ARS_40x_CAN *ars_40x_can);

  ~MotionInputSignalsROS();

 private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  ARS_40x_CAN *ars_40x_can_;

  motion_input_signals::SpeedInformation *speed_information_;

  motion_input_signals::YawRateInformation *yaw_rate_information_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};
}

#endif //ARS_40x_MOTION_INPUT_SIGNALS_ROS_HPP


