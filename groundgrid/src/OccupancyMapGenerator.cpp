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

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <mutex>
#include <fstream>
#include <filesystem>


class OccupancyMapGenerator : public rclcpp::Node {
public:
    OccupancyMapGenerator()
        : Node("occupancy_map_generator"),
          map_initialized_(false)
    {
        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Declare parameters
        this->declare_parameter<double>("resolution", 0.1);
        this->declare_parameter<double>("groundpatch_threshold", 0.3);
        this->declare_parameter<double>("nonground_intensity_threshold", 90.0);
        this->declare_parameter<double>("obstacle_min_radius", 3.0);
        this->declare_parameter<double>("obstacle_max_height", 2.5);
        this->declare_parameter<int>("obstacle_count_threshold", 3);
        this->declare_parameter<double>("free_to_occupied_ratio", 3.0);
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("save_path", "/tmp");
        this->declare_parameter<bool>("use_fixed_bounds", false);
        this->declare_parameter<double>("x_min", -100.0);
        this->declare_parameter<double>("x_max", 100.0);
        this->declare_parameter<double>("y_min", -100.0);
        this->declare_parameter<double>("y_max", 100.0);
        this->declare_parameter<double>("map_size", 200.0);
        this->declare_parameter<std::string>("odom_topic", "/ground_truth");

        // Get parameters
        resolution_ = this->get_parameter("resolution").as_double();
        groundpatch_threshold_ = this->get_parameter("groundpatch_threshold").as_double();
        nonground_intensity_threshold_ = this->get_parameter("nonground_intensity_threshold").as_double();
        obstacle_min_radius_ = this->get_parameter("obstacle_min_radius").as_double();
        obstacle_max_height_ = this->get_parameter("obstacle_max_height").as_double();
        obstacle_count_threshold_ = this->get_parameter("obstacle_count_threshold").as_int();
        free_to_occupied_ratio_ = this->get_parameter("free_to_occupied_ratio").as_double();
        map_frame_ = this->get_parameter("map_frame").as_string();
        save_path_ = this->get_parameter("save_path").as_string();
        use_fixed_bounds_ = this->get_parameter("use_fixed_bounds").as_bool();
        x_min_ = this->get_parameter("x_min").as_double();
        x_max_ = this->get_parameter("x_max").as_double();
        y_min_ = this->get_parameter("y_min").as_double();
        y_max_ = this->get_parameter("y_max").as_double();
        map_size_ = this->get_parameter("map_size").as_double();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();

        // Calculate grid dimensions
        if (use_fixed_bounds_) {
            origin_x_ = x_min_;
            origin_y_ = y_min_;
            grid_width_ = static_cast<int>((x_max_ - x_min_) / resolution_);
            grid_height_ = static_cast<int>((y_max_ - y_min_) / resolution_);
            map_initialized_ = true;
            RCLCPP_INFO(this->get_logger(),
                        "OccupancyMapGenerator using fixed bounds: x=[%.1f, %.1f], y=[%.1f, %.1f], resolution=%.2f",
                        x_min_, x_max_, y_min_, y_max_, resolution_);
        } else {
            grid_width_ = static_cast<int>(map_size_ / resolution_);
            grid_height_ = grid_width_;
            RCLCPP_INFO(this->get_logger(),
                        "OccupancyMapGenerator using dynamic bounds: size=%.1f, resolution=%.2f",
                        map_size_, resolution_);
        }

        // Initialize maps
        free_count_ = cv::Mat::zeros(grid_height_, grid_width_, CV_32SC1);
        occupied_count_ = cv::Mat::zeros(grid_height_, grid_width_, CV_32SC1);

        // Subscribers
        gridmap_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            "groundgrid/grid_map", 1,
            std::bind(&OccupancyMapGenerator::gridMapCallback, this, std::placeholders::_1));

        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "groundgrid/segmented_cloud", 1,
            std::bind(&OccupancyMapGenerator::cloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 1,
            std::bind(&OccupancyMapGenerator::odomCallback, this, std::placeholders::_1));

        // Publishers
        occupancy_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "occupancy_map/image", 1);
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "occupancy_map/grid", 1);

        // Save timer
        save_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&OccupancyMapGenerator::saveTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "OccupancyMapGenerator node initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!map_initialized_) {
            origin_x_ = msg->pose.pose.position.x - map_size_ / 2.0;
            origin_y_ = msg->pose.pose.position.y - map_size_ / 2.0;
            map_initialized_ = true;
            RCLCPP_INFO(this->get_logger(), "Map initialized with origin (%.2f, %.2f)",
                        origin_x_, origin_y_);
        }

        current_pose_ = msg->pose.pose;
    }

    void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!map_initialized_) {
            return;
        }

        grid_map::GridMap gridmap;
        grid_map::GridMapRosConverter::fromMessage(*msg, gridmap);

        if (!gridmap.exists("groundpatch")) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "groundpatch layer not found");
            return;
        }

        const grid_map::Matrix& groundpatch = gridmap["groundpatch"];
        const auto& gridmap_size = gridmap.getSize();
        const float gridmap_resolution = gridmap.getResolution();

        int pixels_per_cell = std::max(1, static_cast<int>(std::ceil(gridmap_resolution / resolution_)));

        for (int i = 0; i < gridmap_size(0); ++i) {
            for (int j = 0; j < gridmap_size(1); ++j) {
                float confidence = groundpatch(i, j);

                if (confidence > groundpatch_threshold_) {
                    grid_map::Position pos;
                    grid_map::Index idx(i, j);
                    if (!gridmap.getPosition(idx, pos)) {
                        continue;
                    }

                    int px_center = static_cast<int>((pos.x() - origin_x_) / resolution_);
                    int py_center = static_cast<int>((pos.y() - origin_y_) / resolution_);

                    int half_pixels = pixels_per_cell / 2;
                    for (int dx = -half_pixels; dx <= half_pixels; ++dx) {
                        for (int dy = -half_pixels; dy <= half_pixels; ++dy) {
                            int px = px_center + dx;
                            int py = py_center + dy;

                            int py_flipped = grid_height_ - 1 - py;

                            if (px >= 0 && px < grid_width_ && py_flipped >= 0 && py_flipped < grid_height_) {
                                free_count_.at<int>(py_flipped, px) += 1;
                            }
                        }
                    }
                }
            }
        }

        publishOccupancyMap();
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!map_initialized_) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        double robot_x = current_pose_.position.x;
        double robot_y = current_pose_.position.y;
        double robot_z = current_pose_.position.z;
        double min_radius_sq = obstacle_min_radius_ * obstacle_min_radius_;

        for (const auto& point : cloud->points) {
            if (point.intensity > nonground_intensity_threshold_) {
                double dx = point.x - robot_x;
                double dy = point.y - robot_y;
                double dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_radius_sq) {
                    continue;
                }

                if (point.z > robot_z + obstacle_max_height_) {
                    continue;
                }

                int px = static_cast<int>((point.x - origin_x_) / resolution_);
                int py = static_cast<int>((point.y - origin_y_) / resolution_);

                py = grid_height_ - 1 - py;

                if (px >= 0 && px < grid_width_ && py >= 0 && py < grid_height_) {
                    occupied_count_.at<int>(py, px) += 1;
                }
            }
        }
    }

    void publishOccupancyMap()
    {
        cv::Mat occupancy_image(grid_height_, grid_width_, CV_8UC1, cv::Scalar(128));

        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                int free_cnt = free_count_.at<int>(y, x);
                int occ_cnt = occupied_count_.at<int>(y, x);

                if (free_cnt > 0 || occ_cnt > 0) {
                    if (free_cnt > occ_cnt * free_to_occupied_ratio_) {
                        occupancy_image.at<uchar>(y, x) = 254;
                    } else if (occ_cnt > obstacle_count_threshold_) {
                        occupancy_image.at<uchar>(y, x) = 0;
                    } else if (free_cnt > 0) {
                        occupancy_image.at<uchar>(y, x) = 254;
                    }
                }
            }
        }

        // Publish image
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = map_frame_;

        auto img_msg = cv_bridge::CvImage(header, "mono8", occupancy_image).toImageMsg();
        occupancy_image_pub_->publish(*img_msg);

        // Publish OccupancyGrid
        nav_msgs::msg::OccupancyGrid grid_msg;
        grid_msg.header = header;
        grid_msg.info.resolution = resolution_;
        grid_msg.info.width = grid_width_;
        grid_msg.info.height = grid_height_;
        grid_msg.info.origin.position.x = origin_x_;
        grid_msg.info.origin.position.y = origin_y_;
        grid_msg.info.origin.position.z = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;

        grid_msg.data.resize(grid_width_ * grid_height_);
        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                int idx = y * grid_width_ + x;
                uchar val = occupancy_image.at<uchar>(grid_height_ - 1 - y, x);

                if (val == 0) {
                    grid_msg.data[idx] = 100;
                } else if (val == 254) {
                    grid_msg.data[idx] = 0;
                } else {
                    grid_msg.data[idx] = -1;
                }
            }
        }
        occupancy_grid_pub_->publish(grid_msg);
    }

    void saveTimerCallback()
    {
        saveMap();
    }

    void saveMap()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!map_initialized_) {
            return;
        }

        // Create directory if it doesn't exist
        try {
            std::filesystem::create_directories(save_path_);
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create directory %s: %s",
                         save_path_.c_str(), e.what());
            return;
        }

        cv::Mat occupancy_image(grid_height_, grid_width_, CV_8UC1, cv::Scalar(205));

        for (int y = 0; y < grid_height_; ++y) {
            for (int x = 0; x < grid_width_; ++x) {
                int free_cnt = free_count_.at<int>(y, x);
                int occ_cnt = occupied_count_.at<int>(y, x);

                if (free_cnt > 0 || occ_cnt > 0) {
                    if (free_cnt > occ_cnt * free_to_occupied_ratio_) {
                        occupancy_image.at<uchar>(y, x) = 254;
                    } else if (occ_cnt > obstacle_count_threshold_) {
                        occupancy_image.at<uchar>(y, x) = 0;
                    } else if (free_cnt > 0) {
                        occupancy_image.at<uchar>(y, x) = 254;
                    }
                }
            }
        }

        // Save PGM
        std::string pgm_path = save_path_ + "/map.pgm";
        bool pgm_saved = cv::imwrite(pgm_path, occupancy_image);
        if (!pgm_saved) {
            RCLCPP_ERROR(this->get_logger(), "Failed to save PGM to %s", pgm_path.c_str());
            return;
        }

        // Save YAML
        std::string yaml_path = save_path_ + "/map.yaml";
        std::ofstream yaml_file(yaml_path);
        if (!yaml_file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file %s", yaml_path.c_str());
            return;
        }
        yaml_file << "image: map.pgm\n";
        yaml_file << "resolution: " << resolution_ << "\n";
        yaml_file << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0.0]\n";
        yaml_file << "negate: 0\n";
        yaml_file << "occupied_thresh: 0.65\n";
        yaml_file << "free_thresh: 0.15\n";
        yaml_file.close();

        RCLCPP_INFO(this->get_logger(), "Map saved to %s (pgm: %s, yaml: %s)",
                    save_path_.c_str(), pgm_path.c_str(), yaml_path.c_str());
    }

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscribers
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr gridmap_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr occupancy_image_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    // Timer
    rclcpp::TimerBase::SharedPtr save_timer_;

    // Parameters
    double resolution_;
    double map_size_;
    double groundpatch_threshold_;
    double nonground_intensity_threshold_;
    double obstacle_min_radius_;
    double obstacle_max_height_;
    int obstacle_count_threshold_;
    double free_to_occupied_ratio_;
    std::string map_frame_;
    std::string save_path_;

    bool use_fixed_bounds_;
    double x_min_, x_max_, y_min_, y_max_;

    // Map data
    int grid_width_, grid_height_;
    cv::Mat free_count_;
    cv::Mat occupied_count_;
    double origin_x_, origin_y_;
    geometry_msgs::msg::Pose current_pose_;
    bool map_initialized_;

    std::mutex mutex_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<OccupancyMapGenerator>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
