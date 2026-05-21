/*
 * Translates sensor_msgs/NavSatFix into nav_msgs/Odometry using UTM
 * ROS2 version
 */
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Dense>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <mutex>
#include <limits>

#include "utils/gpsTransformation.h"

using namespace gps_transfromation;

class GpsTransformationNode : public rclcpp::Node
{
public:
    GpsTransformationNode() : Node("utmtomap_node")
    {
        // Declare parameters
        this->declare_parameter<std::string>("frame_id", "gps_origin");
        this->declare_parameter<std::string>("child_frame_id", "");
        this->declare_parameter<bool>("append_zone", false);
        this->declare_parameter<double>("gps_origin_lat", 0.0);
        this->declare_parameter<double>("gps_origin_long", 0.0);
        this->declare_parameter<double>("gps_origin_alt", 0.0);
        this->declare_parameter<double>("gps_origin_yaw", 0.0);
        this->declare_parameter<std::string>("gps_origin_zone", "");
        this->declare_parameter<bool>("use_initial_altitude_as_origin", false);
        this->declare_parameter<std::vector<double>>("offset", std::vector<double>{0.0, 0.0, 0.0});

        // Get parameters
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("child_frame_id", child_frame_id_);
        this->get_parameter("append_zone", append_zone_);
        this->get_parameter("gps_origin_lat", gps_origin_lat_);
        this->get_parameter("gps_origin_long", gps_origin_long_);
        this->get_parameter("gps_origin_alt", gps_origin_alt_);
        this->get_parameter("gps_origin_yaw", gps_origin_yaw_);
        this->get_parameter("gps_origin_zone", gps_origin_zone_);
        this->get_parameter("use_initial_altitude_as_origin", use_initial_altitude_as_origin_);
        this->get_parameter("offset", offset_);

        // Ensure offset has 3 elements
        if (offset_.size() < 3)
        {
            offset_.resize(3, 0.0);
        }

        // Create publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/gps", 10);

        // Create subscribers
        fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            std::bind(&GpsTransformationNode::fixCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/imu", rclcpp::SensorDataQoS(),
            std::bind(&GpsTransformationNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "\033[1;32m----> GPS Transformation Node Started.\033[0m");
    }

private:
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr fix)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        if (fix->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "No fix.");
            return;
        }

        if (fix->header.stamp.sec == 0 && fix->header.stamp.nanosec == 0)
        {
            return;
        }

        if (fabs(fix->latitude - last_latitude_) <= 1.0e-12 && fabs(fix->longitude - last_longitude_) <= 1.0e-12)
        {
            return;
        }
        last_latitude_ = fix->latitude;
        last_longitude_ = fix->longitude;

        if (initial_altitude_ == std::numeric_limits<double>::max())
        {
            initial_altitude_ = fix->altitude;
        }

        double northing, easting;
        std::string zone;

        LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

        if (gps_origin_zone_ != zone)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "The UTM zones are different. gps origin zone: " << gps_origin_zone_ << " , gps fix zone: " << zone);
            return;
        }

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = fix->header.stamp;

        if (frame_id_.empty())
        {
            if (append_zone_)
            {
                odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
            }
            else
            {
                odom.header.frame_id = fix->header.frame_id;
            }
        }
        else
        {
            if (append_zone_)
            {
                odom.header.frame_id = frame_id_ + "/utm_" + zone;
            }
            else
            {
                odom.header.frame_id = frame_id_;
            }
        }

        // preparing transformation of gps origin
        double gps_origin_northing, gps_origin_easting;
        std::string gps_origin_zone_local;
        LLtoUTM(gps_origin_lat_, gps_origin_long_, gps_origin_northing, gps_origin_easting, gps_origin_zone_local);

        tf2::Transform gps_origin_tf;

        tf2::Quaternion gps_origin_quat;
        gps_origin_quat.setRPY(0.0, 0.0, gps_origin_yaw_);

        if (use_initial_altitude_as_origin_)
        {
            gps_origin_tf.setOrigin(tf2::Vector3(gps_origin_easting, gps_origin_northing, initial_altitude_));
        }
        else
        {
            gps_origin_tf.setOrigin(tf2::Vector3(gps_origin_easting, gps_origin_northing, gps_origin_alt_));
        }

        gps_origin_tf.setRotation(gps_origin_quat);

        // transformation from utm to map
        tf2::Transform pos_utm;
        pos_utm.setRotation(tf2::Quaternion::getIdentity());
        pos_utm.setOrigin(tf2::Vector3(easting, northing, fix->altitude));

        tf2::Transform pos_map;
        pos_map.mult(gps_origin_tf.inverse(), pos_utm);

        tf2::Quaternion pos_map_orientation;

        pos_map_orientation.setRPY(roll_, pitch_, yaw_);
        pos_map.setRotation(pos_map_orientation);

        tf2::Transform sensor_offset;
        tf2::Transform corrected_pos_map;
        if ((offset_[0] || offset_[1] || offset_[2]) && !pose_subscribe_flag_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "The GPS offset is not zero, but the pose is not subscribed. Publish the pose or set offset to zero");
            sensor_offset.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        }
        else
        {
            sensor_offset.setOrigin(tf2::Vector3(offset_[0], offset_[1], offset_[2]));
        }
        sensor_offset.setRotation(tf2::Quaternion::getIdentity());
        corrected_pos_map = pos_map * sensor_offset;

        // Convert tf2::Transform to geometry_msgs::msg::Pose
        odom.pose.pose.position.x = corrected_pos_map.getOrigin().getX();
        odom.pose.pose.position.y = corrected_pos_map.getOrigin().getY();
        odom.pose.pose.position.z = corrected_pos_map.getOrigin().getZ();
        odom.pose.pose.orientation = tf2::toMsg(corrected_pos_map.getRotation());

        odom.child_frame_id = child_frame_id_;

        const int POSE_SIZE = 6;
        const int POSITION_SIZE = 3;

        Eigen::MatrixXd latest_cartesian_covariance;
        latest_cartesian_covariance.resize(POSE_SIZE, POSE_SIZE);
        latest_cartesian_covariance.setZero();

        for (size_t i = 0; i < POSITION_SIZE; i++)
        {
            for (size_t j = 0; j < POSITION_SIZE; j++)
            {
                latest_cartesian_covariance(i, j) = fix->position_covariance[POSITION_SIZE * i + j];
            }
        }
        tf2::Matrix3x3 rot(gps_origin_tf.inverse().getRotation());
        Eigen::MatrixXd rot_6d(POSE_SIZE, POSE_SIZE);

        rot_6d.setIdentity();
        for (size_t rInd = 0; rInd < POSITION_SIZE; ++rInd)
        {
            rot_6d(rInd, 0) = rot.getRow(rInd).getX();
            rot_6d(rInd, 1) = rot.getRow(rInd).getY();
            rot_6d(rInd, 2) = rot.getRow(rInd).getZ();
            rot_6d(rInd + POSITION_SIZE, 3) = rot.getRow(rInd).getX();
            rot_6d(rInd + POSITION_SIZE, 4) = rot.getRow(rInd).getY();
            rot_6d(rInd + POSITION_SIZE, 5) = rot.getRow(rInd).getZ();
        }

        // Rotate the covariance
        latest_cartesian_covariance = rot_6d * latest_cartesian_covariance.eval() * rot_6d.transpose();

        for (size_t i = 0; i < POSE_SIZE; i++)
        {
            for (size_t j = 0; j < POSE_SIZE; j++)
            {
                odom.pose.covariance[POSE_SIZE * i + j] = latest_cartesian_covariance(i, j);
            }
        }
        odom_pub_->publish(odom);
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        tf2::Quaternion orientation;
        tf2::fromMsg(odom_msg->pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll_, pitch_, yaw_);
        pose_subscribe_flag_ = true;
    }

    // Parameters
    std::string frame_id_;
    std::string child_frame_id_;
    bool append_zone_;
    double gps_origin_lat_;
    double gps_origin_long_;
    double gps_origin_alt_;
    double gps_origin_yaw_;
    std::string gps_origin_zone_;
    bool use_initial_altitude_as_origin_;
    std::vector<double> offset_;

    // State variables
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;
    bool pose_subscribe_flag_ = false;
    double last_latitude_ = 0.0;
    double last_longitude_ = 0.0;
    double initial_altitude_ = std::numeric_limits<double>::max();

    std::mutex mtx_;

    // Publishers and subscribers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GpsTransformationNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
