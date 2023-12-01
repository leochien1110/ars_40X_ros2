//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40x_CLUSTER_LIST_ROS_HPP
#define ARS_40x_CLUSTER_LIST_ROS_HPP

#include <rclcpp/rclcpp.hpp>

#include <cstdint>

#include "ars_40x/cluster_list.hpp"
#include "ars_40x/ars_40x_can.hpp"

#include "perception_msgs/msg/cluster_list.hpp"

namespace ars_40x {
class ClusterListROS : public rclcpp::Node {
public:
    ClusterListROS(ARS_40x_CAN *ars_40x_can);

    ~ClusterListROS();

    void set_frame_id(std::string frame_id);

    void send_cluster_0_status();

    void send_cluster_1_general();

    void send_cluster_2_quality();

private:
    std::string frame_id_;

    // ros::Publisher clusters_data_pub_;
    rclcpp::Publisher<perception_msgs::msg::ClusterList>::SharedPtr clusters_data_pub_;

    perception_msgs::msg::ClusterList cluster_list;

    cluster_list::Cluster_0_Status *cluster_0_status_;

    cluster_list::Cluster_1_General *cluster_1_general_;

    cluster_list::Cluster_2_Quality *cluster_2_quality_;

    ARS_40x_CAN *ars_40x_can_;

    int cluster_id_;
};
}

#endif //ARS_40x_CLUSTER_LIST_ROS_HPP
