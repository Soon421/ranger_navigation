#ifndef NAV2_NAVIGATION__GOAL_PUBLISHER_HPP_
#define NAV2_NAVIGATION__GOAL_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav2_navigation/srv/save_waypoints.hpp"
#include "nav2_navigation/srv/load_waypoints.hpp"

#include <vector>
#include <cmath>
#include <string>

// Navigation mode enum
enum class NavigationMode {
    OFF,
    LOOP,        // start -> 1 -> 2 -> ... -> N -> 1 -> 2 -> ...
    ROUND_TRIP   // start -> 1 -> 2 -> ... -> N -> N-1 -> ... -> 1 -> start -> 1 -> ...
};

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher();

private:
    // Callback functions
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void listWaypointsCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void removeLastWaypointCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void clearAllWaypointsCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void autoNavCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void stopNavCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void toggleLoopModeCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response);
    void saveWaypointsCallback(
        const nav2_navigation::srv::SaveWaypoints::Request::SharedPtr request,
        nav2_navigation::srv::SaveWaypoints::Response::SharedPtr response);
    void loadWaypointsCallback(
        const nav2_navigation::srv::LoadWaypoints::Request::SharedPtr request,
        nav2_navigation::srv::LoadWaypoints::Response::SharedPtr response);
    void timerCallback();

    // Helper functions
    void loadWaypointsFromParams();
    void publishCurrentGoal();
    void publishMarkers();
    void updateRobotStartPosition();

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr auto_nav_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_nav_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_loop_mode_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr list_waypoints_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_waypoint_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_all_waypoints_service_;
    rclcpp::Service<nav2_navigation::srv::SaveWaypoints>::SharedPtr save_waypoints_service_;
    rclcpp::Service<nav2_navigation::srv::LoadWaypoints>::SharedPtr load_waypoints_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Waypoints storage
    std::vector<geometry_msgs::msg::PointStamped> waypoints_;

    // Navigation state
    bool is_navigating_;
    NavigationMode navigation_mode_;  // OFF, LOOP, ROUND_TRIP
    bool is_forward_direction_;    // // Forward or reverse direction in round trip mode
    bool going_to_start_;          // Moving to the start position
    size_t current_waypoint_index_;
    double arrival_threshold_;

    // Robot start position for visualization
    geometry_msgs::msg::Point robot_start_position_;
    bool has_robot_start_position_;

    // Waypoint file path
    std::string waypoints_file_path_;

    // Previous number of waypoints (for deleting markers)
    size_t prev_waypoint_count_;
};

#endif  // NAV2_NAVIGATION__GOAL_PUBLISHER_HPP_
