/*
Copyright 2023 Dahlem Center for Machine Learning and Robotics, Freie Universität Berlin

Redistribution and use in source and binary forms, with or without modification, are permitted
provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions
and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of
conditions and the following disclaimer in the documentation and/or other materials provided
with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "velodyne_pointcloud/point_types.h"

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

// Config
#include <groundgrid/GroundGridConfig.hpp>


namespace groundgrid {

/**
 * @brief GroundSegmentation class for ground point detection
 */
class GroundSegmentation {
public:
    using PCLPoint = velodyne_pointcloud::PointXYZIR;

    /**
     * @brief Default constructor
     */
    GroundSegmentation() = default;

    /**
     * @brief Initialize the ground segmentation
     * @param dimension Map dimension in meters
     * @param resolution Map resolution in meters
     */
    void init(const size_t dimension, const float& resolution);

    /**
     * @brief Filter point cloud to separate ground and non-ground points
     * @param cloud Input point cloud
     * @param cloudOrigin Origin of the point cloud
     * @param mapToBase Transform from map to base_link
     * @param map Grid map to use for segmentation
     * @return Filtered point cloud with intensity labels (49=ground, 99=non-ground)
     */
    pcl::PointCloud<PCLPoint>::Ptr filter_cloud(
        const pcl::PointCloud<PCLPoint>::Ptr cloud,
        const PCLPoint& cloudOrigin,
        const geometry_msgs::msg::TransformStamped& mapToBase,
        grid_map::GridMap& map);

    /**
     * @brief Insert cloud points into grid cells
     * @param cloud Input point cloud
     * @param start Start index in cloud
     * @param end End index in cloud
     * @param cloudOrigin Origin of the point cloud
     * @param point_index Output vector of point indices and grid indices
     * @param ignored Output vector of ignored points
     * @param outliers Output vector of outlier indices
     * @param map Grid map
     */
    void insert_cloud(
        const pcl::PointCloud<PCLPoint>::Ptr cloud,
        const size_t start,
        const size_t end,
        const PCLPoint& cloudOrigin,
        std::vector<std::pair<size_t, grid_map::Index>>& point_index,
        std::vector<std::pair<size_t, grid_map::Index>>& ignored,
        std::vector<size_t>& outliers,
        grid_map::GridMap& map);

    /**
     * @brief Set configuration parameters
     * @param config Configuration
     */
    void setConfig(const GroundGridConfig& config);

    /**
     * @brief Detect ground patches in grid map section
     * @param map Grid map
     * @param section Section index (0-3 for parallel processing)
     */
    void detect_ground_patches(grid_map::GridMap& map, unsigned short section) const;

    /**
     * @brief Detect ground patch at specific cell
     * @tparam S Patch size (3 or 5)
     * @param map Grid map
     * @param i Row index
     * @param j Column index
     */
    template<int S>
    void detect_ground_patch(grid_map::GridMap& map, size_t i, size_t j) const;

    /**
     * @brief Interpolate ground heights using spiral pattern
     * @param map Grid map
     * @param toBase Transform to base_link
     */
    void spiral_ground_interpolation(
        grid_map::GridMap& map,
        const geometry_msgs::msg::TransformStamped& toBase) const;

    /**
     * @brief Interpolate single cell height
     * @param map Grid map
     * @param x X index
     * @param y Y index
     */
    void interpolate_cell(grid_map::GridMap& map, const size_t x, const size_t y) const;

protected:
    /// Configuration parameters
    GroundGridConfig mConfig;

    /// Expected points per cell matrix
    grid_map::Matrix expectedPoints;

    /// Velodyne 128: Average distance in rad on the unit circle
    const float verticalPointAngDist = 0.00174532925 * 2;  // 0.2 degrees HDL-64e

    /// Minimum distance squared for point consideration
    const float minDistSquared = 12.0f;
};

}  // namespace groundgrid
