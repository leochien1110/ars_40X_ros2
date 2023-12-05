/**
 * @file cluster_pub.cpp
 * @brief Publish psuedo cluster data for visualization
*/

#include <rclcpp/rclcpp.hpp>

#include "perception_msgs/msg/cluster_list.hpp"

// publisher
rclcpp::Publisher<perception_msgs::msg::ClusterList>::SharedPtr cluster_pub;

void callback()
{
    // Create a cluster list message
    auto cluster_list = std::make_shared<perception_msgs::msg::ClusterList>();
    cluster_list->header.stamp = rclcpp::Clock().now();
    cluster_list->header.frame_id = "radar";

    // Create a cluster message in loop
    for (int i = 0; i < 10; i++)
    {
        auto cluster = std::make_shared<perception_msgs::msg::Cluster>();

        cluster->id = i;

        // random position within x[0, 10], y[-10, 10], z[0, 0]
        cluster->position.pose.position.x = 10 * (rand() / (RAND_MAX + 1.0));
        cluster->position.pose.position.y = 20 * (rand() / (RAND_MAX + 1.0)) - 10;
        cluster->position.pose.position.z = 0;

        // random velocity within 20m/s in x y z
        cluster->relative_velocity.twist.linear.x = 20 * (rand() / (RAND_MAX + 1.0));
        cluster->relative_velocity.twist.linear.y = 20 * (rand() / (RAND_MAX + 1.0));
        cluster->relative_velocity.twist.linear.z = 20 * (rand() / (RAND_MAX + 1.0));

        // random rcs within 10dBm
        cluster->rcs = 10 * (rand() / (RAND_MAX + 1.0));

        // random dynamic property 8 prop
        cluster->dynamic_property = rand() % 8;

        // random probability of existence 8 possibilities
        cluster->prob_of_exist = rand() % 8;

        // random ambiguity state 4 possibilities
        cluster->ambig_state = rand() % 4;

        // random invalid state 18 possibilities
        cluster->invalid_state = rand() % 18;

        // assgin cluster to cluster list
        cluster_list->clusters.push_back(*cluster);
    }

    // publish cluster list
    cluster_pub->publish(*cluster_list);

}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cluster_pub_test");
    cluster_pub = node->create_publisher<perception_msgs::msg::ClusterList>("ars_40x/clusters", 10);
    auto timer = node->create_wall_timer(std::chrono::milliseconds(100), callback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}