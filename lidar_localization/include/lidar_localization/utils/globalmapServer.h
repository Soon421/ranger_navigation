#ifndef GLOBALMAPSERVER_H
#define GLOBALMAPSERVER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree.h>
#include <fstream>
#include <chrono>
#include <string>
#include <std_srvs/srv/empty.hpp>

#include "utils/customVoxelGrid.h"

typedef pcl::PointXYZI PointType;

class GlobalmapServer : public rclcpp::Node {
public:
    GlobalmapServer();

private:
    void load_globalmap();
    void pub_once_cb();
    bool save_map(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> res);

    // ROS2
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxelmap_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srvSaveMap;
    rclcpp::TimerBase::SharedPtr globalmap_pub_timer;

    pcl::PointCloud<PointType>::Ptr globalmap;
    pcl::PointCloud<PointType>::Ptr voxelmap;

    // parameters
    std::string globalmap_pcd;
    float globalmap_downsample_resolution;
    std::vector<float> globalmap_xyzrpy;
    int voxelmap_min_points_per_voxel;
};

#endif // GLOBALMAPSERVER_H
