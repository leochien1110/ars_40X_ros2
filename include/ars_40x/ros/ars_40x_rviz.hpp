//
// Created by shivesh on 9/13/19.
//

#ifndef ARS_40x_ARS_40x_RVIZ_HPP
#define ARS_40x_ARS_40x_RVIZ_HPP

#include <rclcpp/rclcpp.hpp>

namespace ars_40x {
enum {
  POINT,
  CAR,
  TRUCK,
  PEDESTRIAN,
  MOTORCYCLE,
  BICYCLE,
  WIDE,
  RESERVED
};

enum {
  INVALID,
  PERCENT_25,
  PERCENT_50,
  PERCENT_75,
  PERCENT_90,
  PERCENT_99,
  PERCENT_99_9,
  PERCENT_100
};

class ContinentalRadarRViz : public rclcpp::Node {
 public:
  ContinentalRadarRViz();

  ~ContinentalRadarRViz();

 private:
  void clusters_callback(const perception_msgs::msg::ClusterList::SharedPtr cluster_list);

  void objects_callback(const perception_msgs::msg::ObjectList::SharedPtr object_list);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_pub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_pub_;

  rclcpp::Subscription<perception_msgs::msg::ClusterList>::SharedPtr clusters_sub_;

  rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr objects_sub_;
};
}

#endif //ARS_40x_ARS_40x_RVIZ_HPP