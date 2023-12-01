#include <utility>

//
// Created by shivesh on 9/17/19.
//

#include "ars_40x/ros/cluster_list_ros.hpp"

namespace ars_40x {
ClusterListROS::ClusterListROS(ARS_40x_CAN *ars_40x_can) :
    Node("ars_40x_cluster_list_ros"),
    ars_40x_can_(ars_40x_can), 
    cluster_id_(0) {
  cluster_0_status_ = ars_40x_can->get_cluster_0_status();
  cluster_1_general_ = ars_40x_can->get_cluster_1_general();
  cluster_2_quality_ = ars_40x_can->get_cluster_2_quality();
  clusters_data_pub_ = this->create_publisher<perception_msgs::msg::ClusterList>("ars_40x/clusters", 10);
}

ClusterListROS::~ClusterListROS() {
}

void ClusterListROS::set_frame_id(std::string frame_id) {
  frame_id_ = std::move(frame_id);
}

void ClusterListROS::send_cluster_0_status() {
  cluster_list.header.stamp = this->now();
  cluster_list.header.frame_id = frame_id_;
  clusters_data_pub_->publish(cluster_list);
  cluster_id_ = 0;
  cluster_list.clusters.clear();
}

void ClusterListROS::send_cluster_1_general() {
  perception_msgs::msg::Cluster cluster;
  cluster.id = cluster_1_general_->get_cluster_id();
  cluster.position.pose.position.x = cluster_1_general_->get_cluster_long_dist();
  cluster.position.pose.position.y = cluster_1_general_->get_cluster_lat_dist();
  cluster.relative_velocity.twist.linear.x = cluster_1_general_->get_cluster_long_rel_vel();
  cluster.relative_velocity.twist.linear.y = cluster_1_general_->get_cluster_lat_rel_vel();
  cluster.dynamic_property = cluster_1_general_->get_cluster_dyn_prop();
  cluster.rcs = cluster_1_general_->get_cluster_rcs();
  cluster_list.clusters.push_back(cluster);
}

void ClusterListROS::send_cluster_2_quality() {
  cluster_list.clusters[cluster_id_].position.covariance[0] =
      pow(cluster_2_quality_->get_cluster_long_dist_rms(), 2);
  cluster_list.clusters[cluster_id_].position.covariance[7] =
      pow(cluster_2_quality_->get_cluster_lat_dist_rms(), 2);
  cluster_list.clusters[cluster_id_].relative_velocity.covariance[0] =
      pow(cluster_2_quality_->get_cluster_long_rel_vel_rms(), 2);
  cluster_list.clusters[cluster_id_].relative_velocity.covariance[7] =
      pow(cluster_2_quality_->get_cluster_lat_rel_vel_rms(), 2);
  cluster_list.clusters[cluster_id_].prob_of_exist = cluster_2_quality_->get_cluster_pdh0();
  cluster_list.clusters[cluster_id_].ambig_state =
      cluster_2_quality_->get_cluster_ambiguity_state();
  cluster_list.clusters[cluster_id_].invalid_state =
      cluster_2_quality_->get_cluster_validity_state();
  ++cluster_id_;
}
}