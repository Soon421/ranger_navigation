#ifndef MAP_MATCHING_H
#define MAP_MATCHING_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <mutex>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <map>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <pclomp/ndt_omp.h>

#include "utils/customVoxelGrid.h"

using PointType = pcl::PointXYZI;

class MapMatching {
public:
    MapMatching();

    void initializeNDT();
    void align(pcl::PointCloud<PointType>::Ptr& output_cloud, const Eigen::Matrix4f& initial_pose);
    void alignWithParticle(pcl::PointCloud<PointType>::Ptr& output_cloud, const Eigen::Matrix4f& initial_pose,
                           const int& particle_num_trans, const int& particle_num_rot,
                           const double& particle_max_trans, const double& particle_max_rot);
    void setInputTarget(pcl::PointCloud<PointType>::Ptr map);
    void setInputSource(const sensor_msgs::msg::PointCloud2& cloud);
    void setInputSource(const pcl::PointCloud<PointType>::Ptr& scan);
    Eigen::Matrix4f getFinalTransformation() const;

    bool hasConverged() const;
    double getFitnessScore() const;
    double calculateProbability(const pcl::PointCloud<PointType>::Ptr& cloud) const;
    double getOverlapRatio() const;

    // Setters
    void setNDTNeighborSearchMethod(const std::string& method);
    void setTransformationEpsilon(double epsilon);
    void setMaxIteration(int maxIter);
    void setNDTResolution(double resolution);
    void setNumThreads(int numThreads);
    void setLidarDownsampleResolution(double resolution);



private:
    pcl::PointCloud<PointType>::Ptr globalmap_;
    pcl::PointCloud<PointType>::Ptr input_cloud_;
    pclomp::NormalDistributionsTransform<PointType, PointType>::Ptr ndt_;

    pcl::CustomVoxelGrid<PointType> lidar_downsample_filter_;

    // NDT parameters
    std::string ndt_neighbor_search_method_;
    double transformation_elipson_;
    int max_iteration_;
    double ndt_resolution_;
    int number_of_threads_ndt_score_;

    // Map matching parameters
    double lidar_downsample_resolution_;

    static const std::map<std::string, pclomp::NeighborSearchMethod> methodMap_;
};

#endif // MAP_MATCHING_H
