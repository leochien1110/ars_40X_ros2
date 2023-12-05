/**
 * @file service_test.cpp
 * @brief Test for custom service
*/

#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <perception_msgs/srv/max_distance.hpp>

class RadarCfgROS : public rclcpp::Node {
public:
    RadarCfgROS() : Node("radar_cfg_ros") {
        set_max_distance_service_ =
            this->create_service<perception_msgs::srv::MaxDistance>(
                "set_max_distance",
                std::bind(&RadarCfgROS::set_max_distance, this,
                          std::placeholders::_1, std::placeholders::_2));
    }

private:
    void set_max_distance(
        const std::shared_ptr<perception_msgs::srv::MaxDistance::Request> req,
        std::shared_ptr<perception_msgs::srv::MaxDistance::Response> res) {
        // res->success = true;
        RCLCPP_INFO(this->get_logger(), "Request: %d", req->max_distance);
    }

    rclcpp::Service<perception_msgs::srv::MaxDistance>::SharedPtr
        set_max_distance_service_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadarCfgROS>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}