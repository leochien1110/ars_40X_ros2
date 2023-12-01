//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40x_OBJECT_LIST_ROS_HPP
#define ARS_40x_OBJECT_LIST_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

#include "perception_msgs/msg/object_list.hpp"
#include "ars_40x/ars_40x_can.hpp"

namespace ars_40x {
class ObjectListROS : public rclcpp::Node {
 public:
  ObjectListROS(ARS_40x_CAN *continentalRadarCAN);

  ~ObjectListROS();

  void set_frame_id(std::string frame_id);

  void send_object_0_status();

  void send_object_1_general();

  void send_object_2_quality();

  void send_object_3_extended();

 private:
  std::string frame_id_;

  rclcpp::Publisher<perception_msgs::msg::ObjectList>::SharedPtr objects_data_pub_;

  ARS_40x_CAN *ars_40x_can_;

  perception_msgs::msg::ObjectList object_list;

  object_list::Object_0_Status *object_0_status_;

  object_list::Object_1_General *object_1_general_;

  object_list::Object_2_Quality *object_2_quality_;

  object_list::Object_3_Extended *object_3_extended_;

  int object_2_quality_id_;

  int object_3_extended_id_;
};
}

#endif //ARS_40x_OBJECT_LIST_ROS_HPP
