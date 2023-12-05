#include "ars_40x/ros/radar_cfg_ros.hpp"

using namespace perception_msgs::srv;

namespace ars_40x {
RadarCfgROS::RadarCfgROS(ARS_40x_CAN *ars_40x_can) :
    rclcpp::Node("radar_cfg_ros"),
    ars_40x_can_(ars_40x_can) {
    radar_cfg_ = ars_40x_can_->get_radar_cfg();

    set_max_distance_service_ =
        this->create_service<MaxDistance>(
            "set_max_distance",
            std::bind(&RadarCfgROS::set_max_distance, this,
                        std::placeholders::_1, std::placeholders::_2));
    
    set_sensor_id_service_ = 
        this->create_service<SensorID>(
            "set_sensor_id",
            std::bind(&RadarCfgROS::set_sensor_id, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    set_radar_power_service_ =
        this->create_service<RadarPower>(
            "set_radar_power",
            std::bind(&RadarCfgROS::set_radar_power, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    set_output_type_service_ =
        this->create_service<OutputType>(
            "set_output_type",
            std::bind(&RadarCfgROS::set_output_type, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    set_send_quality_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "set_send_quality",
            std::bind(&RadarCfgROS::set_send_quality, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    set_send_ext_info_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "set_send_ext_info",
            std::bind(&RadarCfgROS::set_send_ext_info, this,
                      std::placeholders::_1, std::placeholders::_2));
    
    set_sort_index_service_ =
        this->create_service<SortIndex>(
            "set_sort_index",
            std::bind(&RadarCfgROS::set_sort_index, this,
                std::placeholders::_1, std::placeholders::_2));
    
    set_ctrl_relay_cfg_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "set_ctrl_relay_cfg",
            std::bind(&RadarCfgROS::set_ctrl_relay_cfg, this,
            std::placeholders::_1, std::placeholders::_2));
    
    set_store_in_nvm_service_ =
        this->create_service<std_srvs::srv::SetBool>(
            "set_store_in_nvm",
            std::bind(&RadarCfgROS::set_store_in_nvm, this,
            std::placeholders::_1, std::placeholders::_2));
    
    set_rcs_threshold_service_ =
        this->create_service<RCSThreshold>(
            "set_rcs_threshold",
            std::bind(&RadarCfgROS::set_rcs_threshold, this,
            std::placeholders::_1, std::placeholders::_2));
}

RadarCfgROS::~RadarCfgROS() {
}

void RadarCfgROS::set_max_distance(
    const std::shared_ptr<perception_msgs::srv::MaxDistance::Request> req,
    std::shared_ptr<perception_msgs::srv::MaxDistance::Response> res) {
    if (!radar_cfg_->set_max_distance(static_cast<uint64_t>(req->max_distance))) {
        return;
    }
    ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_sensor_id(
    const std::shared_ptr<SensorID::Request> req,
    std::shared_ptr<SensorID::Response> /*res*/) {
    if (!radar_cfg_->set_sensor_id(req->sensor_id)) {
        return;
    }
    ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_radar_power(
    const std::shared_ptr<RadarPower::Request> req,
    std::shared_ptr<RadarPower::Response> /*res*/) {
  if (!radar_cfg_->set_radar_power(req->radar_power)) {
    return;
  }
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_output_type(
    const std::shared_ptr<OutputType::Request> req,
    std::shared_ptr<OutputType::Response> /*res*/) {
  if (!radar_cfg_->set_output_type(req->output_type)) {
    return;
  }
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_send_quality(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/) {
  radar_cfg_->set_send_quality(static_cast<bool>(req->data));
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_send_ext_info(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/) {
  radar_cfg_->set_send_ext_info(static_cast<bool>(req->data));
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_sort_index(
    const std::shared_ptr<SortIndex::Request> req,
    std::shared_ptr<SortIndex::Response> /*res*/) {
  if (!radar_cfg_->set_sort_index(req->sort_index)) {
    return;
  }
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_ctrl_relay_cfg(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/) {
  radar_cfg_->set_ctrl_relay_cfg(static_cast<bool>(req->data));
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_store_in_nvm(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/) {
  radar_cfg_->set_store_in_nvm(static_cast<bool>(req->data));
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

void RadarCfgROS::set_rcs_threshold(
    const std::shared_ptr<RCSThreshold::Request> req,
    std::shared_ptr<RCSThreshold::Response> /*res*/) {
  if (!radar_cfg_->set_rcs_threshold(req->rcs_threshold)) {
    return;
  }
  ars_40x_can_->send_radar_data(can_messages::RadarCfg);
}

}