//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40X_OBSTACLE_ARRAY_HPP
#define ARS_40X_OBSTACLE_ARRAY_HPP

#include <rclcpp/rclcpp.hpp>

#include "perception_msgs/msg/object_list.hpp"

#include "costmap_converter_msgs/msg/obstacle_array_msg.hpp"

namespace ars_40x {
class ObstacleArray : public rclcpp::Node {
 public:
  explicit ObstacleArray();

  ~ObstacleArray();

 private:
  void object_list_callback(const perception_msgs::msg::ObjectList::SharedPtr object_list);

  rclcpp::Publisher<costmap_converter_msgs::msg::ObstacleArrayMsg>::SharedPtr obstacle_array_pub_;

  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr object_list_sub_;
};
}

#endif //ARS_40X_OBSTACLE_ARRAY_HPP