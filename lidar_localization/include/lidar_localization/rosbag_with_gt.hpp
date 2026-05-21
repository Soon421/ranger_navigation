#ifndef ROSBAG_WITH_GT_HPP
#define ROSBAG_WITH_GT_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <Eigen/Dense>
#include <pcl/common/transforms.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

class RosbagWithGT {
public:
    RosbagWithGT(const std::string& ground_truth_topic, const std::string& odometry_topic);
    ~RosbagWithGT();

    bool readRosbag(const std::string& input_bag_path);
    bool writeRosbag(const std::string& output_bag_path);

    nav_msgs::msg::Path getGroundtruthPath();
    Eigen::Matrix4f odom2matrix(const nav_msgs::msg::Odometry& odom);
    nav_msgs::msg::Odometry matrix2odom(const Eigen::Matrix4f& matrix);
    double odomDistance(const nav_msgs::msg::Odometry& odom_1, const nav_msgs::msg::Odometry& odom_2);
    double odomHeadingAngleDifference(const nav_msgs::msg::Odometry& odom_1, const nav_msgs::msg::Odometry& odom_2);
    void progressBar(const int index, const int max);

private:
    struct BagMessage {
        std::string topic;
        std::string type;
        rclcpp::Time timestamp;
        std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
    };

    std::vector<BagMessage> messages_;
    std::unordered_map<std::string, std::string> topic_types_;
    std::vector<rosbag2_storage::TopicMetadata> topic_metadata_list_;

    std::string groundtruth_topic_;
    std::string odometry_topic_;
    std::string modified_odometry_topic_;

    rclcpp::Serialization<nav_msgs::msg::Path> path_serializer_;
    rclcpp::Serialization<nav_msgs::msg::Odometry> odom_serializer_;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer_;
};

#endif // ROSBAG_WITH_GT_HPP
