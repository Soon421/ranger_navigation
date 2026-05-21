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

#include <numeric>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <groundgrid/GroundGrid.hpp>
#include <groundgrid/GroundGridConfig.hpp>
#include <groundgrid/GroundGridFwd.hpp>
#include <groundgrid/GroundSegmentation.hpp>


namespace groundgrid {

class GroundGridNode : public rclcpp::Node {
public:
    using PCLPoint = velodyne_pointcloud::PointXYZIR;

    GroundGridNode()
        : Node("groundgrid")
    {
        mTfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);

        declareParameters();

        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            "groundgrid/grid_map", 1);
        filtered_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "groundgrid/segmented_cloud", 1);

        groundgrid_ = std::make_shared<GroundGrid>(this->get_clock());

        loadConfig();

        ground_segmentation_.init(groundgrid_->mDimension, groundgrid_->mResolution);
        ground_segmentation_.setConfig(config_);

        sensor_frame_ = this->get_parameter("sensor_frame").as_string();

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string points_topic = this->get_parameter("points_topic").as_string();

        pos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::QoS(10).best_effort(),
            std::bind(&GroundGridNode::odomCallback, this, std::placeholders::_1));

        points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic, rclcpp::QoS(10).best_effort(),
            std::bind(&GroundGridNode::pointsCallback, this, std::placeholders::_1));

        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&GroundGridNode::parameterCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribing to odom: %s", odom_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribing to points: %s", points_topic.c_str());
    }

private:
    void declareParameters()
    {
        this->declare_parameter<std::string>("sensor_frame", "velodyne");
        this->declare_parameter<std::string>("odom_topic", "/localization/odometry/filtered_map");
        this->declare_parameter<std::string>("points_topic", "/sensors/velodyne_points");

        this->declare_parameter<int>("point_count_cell_variance_threshold", 10);
        this->declare_parameter<int>("max_ring", 1024);
        this->declare_parameter<double>("groundpatch_detection_minimum_threshold", 0.01);
        this->declare_parameter<double>("distance_factor", 0.00002);
        this->declare_parameter<double>("minimum_distance_factor", 0.0001);
        this->declare_parameter<double>("miminum_point_height_threshold", 0.05);
        this->declare_parameter<double>("minimum_point_height_obstacle_threshold", 0.02);
        this->declare_parameter<double>("outlier_tolerance", 0.03);
        this->declare_parameter<double>("ground_patch_detection_minimum_point_count_threshold", 0.25);
        this->declare_parameter<double>("patch_size_change_distance", 20.0);
        this->declare_parameter<double>("occupied_cells_decrease_factor", 5.0);
        this->declare_parameter<double>("occupied_cells_point_count_factor", 20.0);
        this->declare_parameter<double>("min_outlier_detection_ground_confidence", 1.25);
        this->declare_parameter<int>("thread_count", 8);
    }

    void loadConfig()
    {
        config_.point_count_cell_variance_threshold =
            this->get_parameter("point_count_cell_variance_threshold").as_int();
        config_.max_ring = this->get_parameter("max_ring").as_int();
        config_.groundpatch_detection_minimum_threshold =
            this->get_parameter("groundpatch_detection_minimum_threshold").as_double();
        config_.distance_factor = this->get_parameter("distance_factor").as_double();
        config_.minimum_distance_factor = this->get_parameter("minimum_distance_factor").as_double();
        config_.miminum_point_height_threshold =
            this->get_parameter("miminum_point_height_threshold").as_double();
        config_.minimum_point_height_obstacle_threshold =
            this->get_parameter("minimum_point_height_obstacle_threshold").as_double();
        config_.outlier_tolerance = this->get_parameter("outlier_tolerance").as_double();
        config_.ground_patch_detection_minimum_point_count_threshold =
            this->get_parameter("ground_patch_detection_minimum_point_count_threshold").as_double();
        config_.patch_size_change_distance =
            this->get_parameter("patch_size_change_distance").as_double();
        config_.occupied_cells_decrease_factor =
            this->get_parameter("occupied_cells_decrease_factor").as_double();
        config_.occupied_cells_point_count_factor =
            this->get_parameter("occupied_cells_point_count_factor").as_double();
        config_.min_outlier_detection_ground_confidence =
            this->get_parameter("min_outlier_detection_ground_confidence").as_double();
        config_.thread_count = this->get_parameter("thread_count").as_int();

        groundgrid_->setConfig(config_);
        ground_segmentation_.setConfig(config_);
    }

    rcl_interfaces::msg::SetParametersResult parameterCallback(
        const std::vector<rclcpp::Parameter>& parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "point_count_cell_variance_threshold") {
                config_.point_count_cell_variance_threshold = param.as_int();
            } else if (param.get_name() == "max_ring") {
                config_.max_ring = param.as_int();
            } else if (param.get_name() == "groundpatch_detection_minimum_threshold") {
                config_.groundpatch_detection_minimum_threshold = param.as_double();
            } else if (param.get_name() == "distance_factor") {
                config_.distance_factor = param.as_double();
            } else if (param.get_name() == "minimum_distance_factor") {
                config_.minimum_distance_factor = param.as_double();
            } else if (param.get_name() == "miminum_point_height_threshold") {
                config_.miminum_point_height_threshold = param.as_double();
            } else if (param.get_name() == "minimum_point_height_obstacle_threshold") {
                config_.minimum_point_height_obstacle_threshold = param.as_double();
            } else if (param.get_name() == "outlier_tolerance") {
                config_.outlier_tolerance = param.as_double();
            } else if (param.get_name() == "ground_patch_detection_minimum_point_count_threshold") {
                config_.ground_patch_detection_minimum_point_count_threshold = param.as_double();
            } else if (param.get_name() == "patch_size_change_distance") {
                config_.patch_size_change_distance = param.as_double();
            } else if (param.get_name() == "occupied_cells_decrease_factor") {
                config_.occupied_cells_decrease_factor = param.as_double();
            } else if (param.get_name() == "occupied_cells_point_count_factor") {
                config_.occupied_cells_point_count_factor = param.as_double();
            } else if (param.get_name() == "min_outlier_detection_ground_confidence") {
                config_.min_outlier_detection_ground_confidence = param.as_double();
            } else if (param.get_name() == "thread_count") {
                config_.thread_count = param.as_int();
            }
        }

        groundgrid_->setConfig(config_);
        ground_segmentation_.setConfig(config_);

        return result;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr inOdom)
    {
        map_ptr_ = groundgrid_->update(inOdom);
    }

    void pointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud(
            new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        geometry_msgs::msg::TransformStamped mapToBaseTransform, cloudOriginTransform;

        if (!map_ptr_) {
            return;
        }

        tf2::TimePoint cloud_time = tf2::timeFromSec(
            rclcpp::Time(cloud_msg->header.stamp).seconds());
        try {
            mapToBaseTransform = mTfBuffer->lookupTransform(
                "map", "base_link", cloud_time, tf2::durationFromSec(0.1));
            cloudOriginTransform = mTfBuffer->lookupTransform(
                "map", sensor_frame_, cloud_time, tf2::durationFromSec(0.1));
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(),
                        "Received point cloud but transforms are not available: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PointStamped origin;
        origin.header = cloud_msg->header;
        origin.header.frame_id = sensor_frame_;
        origin.point.x = 0.0f;
        origin.point.y = 0.0f;
        origin.point.z = 0.0f;

        tf2::doTransform(origin, origin, cloudOriginTransform);

        if (cloud_msg->header.frame_id != "map") {
            geometry_msgs::msg::TransformStamped transformStamped;
            pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr transformed_cloud(
                new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
            transformed_cloud->header = cloud->header;
            transformed_cloud->header.frame_id = "map";
            transformed_cloud->points.reserve(cloud->points.size());

            try {
                transformStamped = mTfBuffer->lookupTransform(
                    "map", cloud_msg->header.frame_id, cloud_time, tf2::durationFromSec(0.1));
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(),
                            "Failed to get map transform: %s", ex.what());
                return;
            }

            geometry_msgs::msg::PointStamped psIn;
            psIn.header = cloud_msg->header;

            for (const auto& point : cloud->points) {
                psIn.point.x = point.x;
                psIn.point.y = point.y;
                psIn.point.z = point.z;

                tf2::doTransform(psIn, psIn, transformStamped);

                PCLPoint& point_transformed = transformed_cloud->points.emplace_back(point);
                point_transformed.x = psIn.point.x;
                point_transformed.y = psIn.point.y;
                point_transformed.z = psIn.point.z;
            }

            cloud = transformed_cloud;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg_out;
        PCLPoint origin_pclPoint;
        origin_pclPoint.x = origin.point.x;
        origin_pclPoint.y = origin.point.y;
        origin_pclPoint.z = origin.point.z;

        auto filtered_cloud = ground_segmentation_.filter_cloud(
            cloud, origin_pclPoint, mapToBaseTransform, *map_ptr_);

        pcl::toROSMsg(*filtered_cloud, cloud_msg_out);

        cloud_msg_out.header = cloud_msg->header;
        cloud_msg_out.header.frame_id = "map";
        filtered_cloud_pub_->publish(cloud_msg_out);

        auto grid_map_msg = grid_map::GridMapRosConverter::toMessage(*map_ptr_);
        grid_map_msg->header.stamp = cloud_msg->header.stamp;
        grid_map_pub_->publish(std::move(grid_map_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pos_sub_;

    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    GroundGridConfig config_;
    GroundGridPtr groundgrid_;
    std::shared_ptr<grid_map::GridMap> map_ptr_;
    GroundSegmentation ground_segmentation_;

    std::shared_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;

    std::string sensor_frame_;
};

}  // namespace groundgrid


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<groundgrid::GroundGridNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
