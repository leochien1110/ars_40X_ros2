//
// Created by shivesh on 9/14/19.
//

#ifndef ARS_40X_RADAR_CFG_ROS_HPP
#define ARS_40X_RADAR_CFG_ROS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <cstdint>

#include "ars_40x/ars_40x_can.hpp"
#include "perception_msgs/srv/max_distance.hpp"
#include "perception_msgs/srv/output_type.hpp"
#include "perception_msgs/srv/radar_power.hpp"
#include "perception_msgs/srv/rcs_threshold.hpp"
#include "perception_msgs/srv/sensor_id.hpp"
#include "perception_msgs/srv/sort_index.hpp"

using namespace perception_msgs::srv;

namespace ars_40x {
class RadarCfgROS : public rclcpp::Node {
public:
    explicit RadarCfgROS(ARS_40x_CAN *ars_40x_can);
    explicit RadarCfgROS();

    ~RadarCfgROS();

    void set_max_distance(
        const std::shared_ptr<perception_msgs::srv::MaxDistance::Request> req,
        std::shared_ptr<perception_msgs::srv::MaxDistance::Response> res);

    void set_sensor_id(
        const std::shared_ptr<SensorID::Request> req,
        std::shared_ptr<SensorID::Response> /*res*/);

    void set_radar_power(
        const std::shared_ptr<RadarPower::Request> req,
        std::shared_ptr<RadarPower::Response> /*res*/);

    void set_output_type(
        const std::shared_ptr<OutputType::Request> req,
        std::shared_ptr<OutputType::Response> /*res*/);

    void set_send_quality(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/);

    void set_send_ext_info(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/);

    void set_sort_index(
        const std::shared_ptr<SortIndex::Request> req,
        std::shared_ptr<SortIndex::Response> /*res*/);

    void set_ctrl_relay_cfg(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/);

    void set_store_in_nvm(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> /*res*/);

    void set_rcs_threshold(
        const std::shared_ptr<RCSThreshold::Request> req,
        std::shared_ptr<RCSThreshold::Response> /*res*/);

private:
    ARS_40x_CAN *ars_40x_can_;

    radar_cfg::RadarCfg *radar_cfg_;

    rclcpp::Service<perception_msgs::srv::MaxDistance>::SharedPtr set_max_distance_service_;

    rclcpp::Service<SensorID>::SharedPtr set_sensor_id_service_;

    rclcpp::Service<RadarPower>::SharedPtr set_radar_power_service_;

    rclcpp::Service<OutputType>::SharedPtr set_output_type_service_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_send_quality_service_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_send_ext_info_service_;

    rclcpp::Service<SortIndex>::SharedPtr set_sort_index_service_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_ctrl_relay_cfg_service_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_store_in_nvm_service_;

    rclcpp::Service<RCSThreshold>::SharedPtr set_rcs_threshold_service_;
};
}

#endif //ARS_40X_RADAR_CFG_ROS_HPP