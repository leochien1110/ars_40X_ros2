#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ars_40x/ros/cluster_list_ros.hpp"
#include "ars_40x/ros/motion_input_signals_ros.hpp"
#include "ars_40x/ros/object_list_ros.hpp"
#include "ars_40x/ros/radar_cfg_ros.hpp"
#include "ars_40x/ros/radar_state_ros.hpp"

#include "ars_40x/ars_40x_can.hpp"

namespace ars_40x {
class ARS_40x_ROS : public rclcpp::Node, public ARS_40x_CAN {
 public:
  ARS_40x_ROS();

  ~ARS_40x_ROS();

  void receive_data();

  void run();

  void send_cluster_0_status();

  void send_cluster_1_general();

  void send_cluster_2_quality();

  void send_object_0_status();

  void send_object_1_general();

  void send_object_2_quality();

  void send_object_3_extended();

  void send_radar_state();

 private:
  std::thread receive_data_thread_;

  ClusterListROS cluster_list_ros_;

  MotionInputSignalsROS motion_input_signals_ros_;

  ObjectListROS object_list_ros_;

  RadarCfgROS radar_cfg_ros_;

  RadarStateROS radar_state_ros_;
};
}
