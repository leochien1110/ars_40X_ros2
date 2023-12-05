/**
 * @file ars_40x_rviz.hpp
 * @brief ARS 408/ 410 radar rviz visualization conversion class
 * @date Dec 5, 2023
 * @version 0.1.0
 * @authors shiesh(9/13/19), Wen-Yu Chien
*/

#ifndef ARS_40x_ARS_40x_RVIZ_HPP
#define ARS_40x_ARS_40x_RVIZ_HPP


#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "perception_msgs/msg/cluster_list.hpp"
#include "perception_msgs/msg/object_list.hpp"
#include "pcl_conversions/pcl_conversions.h"

// pcl
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

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

// x, y, z, intensity, velocity, timestamp
struct PointCloudRadar {
    PCL_ADD_POINT4D;            // preferred way of adding a XYZ+padding
    float intensity;            // RCS in dBm
    float velocity;             // velocity in m/s
    float vx;                   // velocity in x direction
    float vy;                   // velocity in y direction
    int8_t dynamic_property;    // dynamic property
    int8_t cls;               // object class
    int8_t prob_of_exist;       // probability of existence
    int8_t ambig_state;         // ambiguity state
    int8_t invalid_state;       // invalid state
    PCL_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;

typedef PointCloudRadar PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef sensor_msgs::msg::PointCloud2 PointCloudMsgT;

class ContinentalRadarRViz : public rclcpp::Node {
public:
    explicit ContinentalRadarRViz();

    ~ContinentalRadarRViz();

private:
    void clusters_callback(const perception_msgs::msg::ClusterList::SharedPtr cluster_list);

    void objects_callback(const perception_msgs::msg::ObjectList::SharedPtr object_list);

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clusters_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr objects_pub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr velocity_pub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc2_pub_;

    rclcpp::Subscription<perception_msgs::msg::ClusterList>::SharedPtr clusters_sub_;

    rclcpp::Subscription<perception_msgs::msg::ObjectList>::SharedPtr objects_sub_;
};
}   // namespace ars_40x

POINT_CLOUD_REGISTER_POINT_STRUCT (ars_40x::PointCloudRadar,           // here we assume a XYZ + "radar_props" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (float, velocity, velocity)
                                   (float, vx, vx)
                                   (float, vy, vy)
                                   (int8_t, dynamic_property, dynamic_property)
                                   (int8_t, cls, cls)
                                   (int8_t, prob_of_exist, prob_of_exist)
                                   (int8_t, ambig_state, ambig_state)
                                   (int8_t, invalid_state, invalid_state)
)

#endif //ARS_40x_ARS_40x_RVIZ_HPP