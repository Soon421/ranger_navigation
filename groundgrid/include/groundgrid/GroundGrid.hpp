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

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// ROS2 msgs
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Config
#include <groundgrid/GroundGridConfig.hpp>


namespace groundgrid {

/**
 * @brief GroundGrid class for managing the 2D grid map and terrain estimation
 */
class GroundGrid {
public:
    /**
     * @brief Constructor
     * @param clock Shared pointer to ROS2 clock for TF2 buffer
     */
    explicit GroundGrid(rclcpp::Clock::SharedPtr clock);

    /**
     * @brief Destructor
     */
    virtual ~GroundGrid();

    /**
     * @brief Sets the current dynamic configuration
     * @param config Configuration parameters
     */
    void setConfig(const GroundGridConfig& config);

    /**
     * @brief Initialize the ground grid from odometry
     * @param inOdom Odometry message
     */
    void initGroundGrid(const nav_msgs::msg::Odometry::SharedPtr inOdom);

    /**
     * @brief Update the grid map based on new odometry
     * @param inOdom Odometry message
     * @return Shared pointer to the updated grid map
     */
    std::shared_ptr<grid_map::GridMap> update(const nav_msgs::msg::Odometry::SharedPtr inOdom);

    /// Grid resolution in meters
    const float mResolution = 0.33f;

    /// Grid dimension in meters
    const float mDimension = 120.0f;

private:
    /// Dynamic config attribute
    GroundGridConfig config_;

    /// TF2 buffer
    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;

    /// TF2 listener
    std::shared_ptr<tf2_ros::TransformListener> mTf2_listener;

    /// Detection radius
    double mDetectionRadius = 60.0;

    /// Grid map pointer
    std::shared_ptr<grid_map::GridMap> mMap_ptr;

    /// Transform stamps
    geometry_msgs::msg::TransformStamped mTfPosition, mTfLux, mTfUtm, mTfMap;

    /// Last pose
    geometry_msgs::msg::PoseWithCovarianceStamped mLastPose;

    /// Logger for ROS2 logging
    rclcpp::Logger logger_;
};

}  // namespace groundgrid
