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

#include <groundgrid/GroundGrid.hpp>

#include <chrono>

// Grid map
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_core/GridMapMath.hpp>

// TF2
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace groundgrid {

GroundGrid::GroundGrid(rclcpp::Clock::SharedPtr clock)
    : logger_(rclcpp::get_logger("groundgrid"))
{
    mTfBuffer = std::make_shared<tf2_ros::Buffer>(clock);
    mTf2_listener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
}

GroundGrid::~GroundGrid() {}

void GroundGrid::setConfig(const GroundGridConfig& config)
{
    config_ = config;
}

void GroundGrid::initGroundGrid(const nav_msgs::msg::Odometry::SharedPtr inOdom)
{
    auto start = std::chrono::steady_clock::now();
    geometry_msgs::msg::PoseWithCovarianceStamped odomPose, mapPose;

    mMap_ptr = std::make_shared<grid_map::GridMap>(
        std::vector<std::string>{"points", "ground", "groundpatch", "minGroundHeight", "maxGroundHeight"});

    grid_map::GridMap& map = *mMap_ptr;
    map.setFrameId("map");
    map.setGeometry(
        grid_map::Length(mDimension, mDimension),
        mResolution,
        grid_map::Position(inOdom->pose.pose.position.x, inOdom->pose.pose.position.y));

    RCLCPP_INFO(logger_, "Created map with size %f x %f m (%i x %i cells).",
                map.getLength().x(), map.getLength().y(),
                map.getSize()(0), map.getSize()(1));

    odomPose.pose = inOdom->pose;
    odomPose.header = inOdom->header;
    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(odomPose.pose.pose.position.x, odomPose.pose.pose.position.y), damage);
    grid_map::BufferRegion region(grid_map::Index(0, 0), map.getSize(), grid_map::BufferRegion::Quadrant(0));

    map["points"].setZero();
    map["ground"].setConstant(inOdom->pose.pose.position.z);
    map["groundpatch"].setConstant(0.0000001);
    map["minGroundHeight"].setConstant(100.0);
    map["maxGroundHeight"].setConstant(-100.0);

    auto end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(logger_, "transforms lookup %ld ms",
                 std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    mLastPose = odomPose;
}

std::shared_ptr<grid_map::GridMap> GroundGrid::update(const nav_msgs::msg::Odometry::SharedPtr inOdom)
{
    if (!mMap_ptr) {
        initGroundGrid(inOdom);
        return mMap_ptr;
    }

    auto start = std::chrono::steady_clock::now();
    grid_map::GridMap& map = *mMap_ptr;

    geometry_msgs::msg::PoseWithCovarianceStamped poseDiff;
    poseDiff.pose.pose.position.x = inOdom->pose.pose.position.x - mLastPose.pose.pose.position.x;
    poseDiff.pose.pose.position.y = inOdom->pose.pose.position.y - mLastPose.pose.pose.position.y;
    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(inOdom->pose.pose.position.x, inOdom->pose.pose.position.y), damage);

    // Static so if the new transform is not yet available, we can use the last one
    static geometry_msgs::msg::TransformStamped base_to_map;

    try {
        base_to_map = mTfBuffer->lookupTransform(
            "base_link", "map",
            tf2::TimePointZero);
    }
    catch (tf2::LookupException& e) {
        // Potentially degraded performance
        RCLCPP_WARN(logger_, "no transform? -> error: %s", e.what());
    }
    catch (tf2::ExtrapolationException& e) {
        // Can happen when new transform has not yet been published, we can use the old one instead
        RCLCPP_DEBUG(logger_, "need to extrapolate a transform? -> error: %s", e.what());
    }

    geometry_msgs::msg::PointStamped ps;
    ps.header = inOdom->header;
    ps.header.frame_id = "map";
    grid_map::Position pos;

    for (auto region : damage) {
        for (auto it = grid_map::SubmapIterator(map, region); !it.isPastEnd(); ++it) {
            auto idx = *it;

            map.getPosition(idx, pos);
            ps.point.x = pos(0);
            ps.point.y = pos(1);
            ps.point.z = 0;
            tf2::doTransform(ps, ps, base_to_map);
            map.at("ground", idx) = -ps.point.z;
            map.at("groundpatch", idx) = 0.0;
        }
    }

    // We haven't moved so we have nothing to do
    if (damage.empty()) {
        return mMap_ptr;
    }

    mLastPose.pose = inOdom->pose;
    mLastPose.header = inOdom->header;

    map.convertToDefaultStartIndex();
    auto end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(logger_, "total %ld ms",
                 std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
    return mMap_ptr;
}

}  // namespace groundgrid
