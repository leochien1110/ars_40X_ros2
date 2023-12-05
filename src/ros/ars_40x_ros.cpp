#include "ars_40x/ros/ars_40x_ros.hpp"

namespace ars_40x {
ARS_40x_ROS::ARS_40x_ROS() :
    rclcpp::Node("ars_40x_ros")
    // cluster_list_ros_(this),
    // motion_input_signals_ros_(this),
    // object_list_ros_(this),
    // radar_cfg_ros_(this),
    // radar_state_ros_(this) 
{
    // spin nodes
    cluster_list_ros_ = std::make_shared<ClusterListROS>(this);
    motion_input_signals_ros_ = std::make_shared<MotionInputSignalsROS>(this);
    object_list_ros_ = std::make_shared<ObjectListROS>(this);
    radar_cfg_ros_ = std::make_shared<RadarCfgROS>();
    radar_state_ros_ = std::make_shared<RadarStateROS>(this);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(cluster_list_ros_);
    executor.add_node(motion_input_signals_ros_);
    executor.add_node(object_list_ros_);
    executor.add_node(radar_cfg_ros_);
    executor.add_node(radar_state_ros_);

    executor.spin();

    std::string frame_id = this->declare_parameter<std::string>("frame_id", "radar");
    cluster_list_ros_->set_frame_id(frame_id);
    object_list_ros_->set_frame_id(frame_id);
}

ARS_40x_ROS::~ARS_40x_ROS() {
}


void ARS_40x_ROS::receive_data() {
    while (rclcpp::ok()) {
        receive_radar_data();
    }
}

void ARS_40x_ROS::send_cluster_0_status() {
    // cluster_list_ros_.send_cluster_0_status();
    cluster_list_ros_->send_cluster_0_status();
}

void ARS_40x_ROS::send_cluster_1_general() {
    // cluster_list_ros_.send_cluster_1_general();
    cluster_list_ros_->send_cluster_1_general();
}

void ARS_40x_ROS::send_cluster_2_quality() {
    // cluster_list_ros_.send_cluster_2_quality();
    cluster_list_ros_->send_cluster_2_quality();
}

void ARS_40x_ROS::send_object_0_status() {
    // object_list_ros_.send_object_0_status();
    object_list_ros_->send_object_0_status();
}

void ARS_40x_ROS::send_object_1_general() {
    // object_list_ros_.send_object_1_general();
    object_list_ros_->send_object_1_general();
}

void ARS_40x_ROS::send_object_2_quality() {
    // object_list_ros_.send_object_2_quality();
    object_list_ros_->send_object_2_quality();
}

void ARS_40x_ROS::send_object_3_extended() {
    // object_list_ros_.send_object_3_extended();
    object_list_ros_->send_object_3_extended();
}

void ARS_40x_ROS::send_radar_state() {
    // radar_state_ros_.send_radar_state();
    radar_state_ros_->send_radar_state();
}

void ARS_40x_ROS::run() {
    receive_data_thread_ = std::thread(std::bind(&ARS_40x_ROS::receive_data, this));
    receive_data_thread_.detach();
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto ars_40x_ros_ = std::make_shared<ars_40x::ARS_40x_ROS>();
    ars_40x_ros_->run();
    rclcpp::spin(ars_40x_ros_);
    rclcpp::shutdown();
}