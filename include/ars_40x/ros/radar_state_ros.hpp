//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40x_RADAR_STATE_ROS_HPP
#define ARS_40x_RADAR_STATE_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

#include "ars_40x/ars_40x_can.hpp"
#include "perception_msgs/msg/radar_status.hpp"

namespace ars_40x {
class RadarStateROS : public rclcpp::Node {
public:
  explicit RadarStateROS(ARS_40x_CAN *ars_40x_can);

  ~RadarStateROS();

  void send_radar_state();

 private:
  rclcpp::Publisher<perception_msgs::msg::RadarStatus>::SharedPtr radar_state_pub_;

  ARS_40x_CAN *ars_40x_can_;

  radar_state::RadarState *radar_state_;
};
}

#endif //ARS_40x_RADAR_STATE_ROS_HPP