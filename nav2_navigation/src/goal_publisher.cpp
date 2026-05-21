#include "nav2_navigation/goal_publisher.hpp"

#include <yaml-cpp/yaml.h>
#include <fstream>
#include <thread>
#include <rclcpp/qos.hpp>

GoalPublisher::GoalPublisher()
: Node("goal_publisher"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
    // Parameters
    this->declare_parameter<double>("arrival_threshold", 1.0);  // meters
    this->declare_parameter<double>("timer_period", 0.1);       // seconds
    this->declare_parameter<std::vector<double>>("waypoints_x", std::vector<double>{});
    this->declare_parameter<std::vector<double>>("waypoints_y", std::vector<double>{});
    this->declare_parameter<std::string>("waypoints_file", "waypoints.yaml");

    arrival_threshold_ = this->get_parameter("arrival_threshold").as_double();
    double timer_period = this->get_parameter("timer_period").as_double();
    waypoints_file_path_ = this->get_parameter("waypoints_file").as_string();

    // Load waypoints from parameters
    loadWaypointsFromParams();

    // Subscriber for clicked_point (waypoints)
    clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10,
        std::bind(&GoalPublisher::clickedPointCallback, this, std::placeholders::_1));

    // Publisher for goal_pose (QoS settings compatible with Nav2)
    auto goal_qos = rclcpp::QoS(rclcpp::KeepLast(1))
        .reliable()
        .durability_volatile();
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", goal_qos);

    // Publisher for visualization markers
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoint_markers", 10);

    // Service for starting auto navigation
    auto_nav_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/auto_nav",
        std::bind(&GoalPublisher::autoNavCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for stopping navigation
    stop_nav_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/stop_nav",
        std::bind(&GoalPublisher::stopNavCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for toggling loop mode
    toggle_loop_mode_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/toggle_loop_mode",
        std::bind(&GoalPublisher::toggleLoopModeCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for listing waypoints
    list_waypoints_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/list_waypoints",
        std::bind(&GoalPublisher::listWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for removing last waypoint
    remove_last_waypoint_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/remove_last_waypoint",
        std::bind(&GoalPublisher::removeLastWaypointCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for clearing all waypoints
    clear_all_waypoints_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/clear_all_waypoints",
        std::bind(&GoalPublisher::clearAllWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for saving waypoints to YAML file
    save_waypoints_service_ = this->create_service<nav2_navigation::srv::SaveWaypoints>(
        "/save_waypoints",
        std::bind(&GoalPublisher::saveWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Service for loading waypoints from YAML file
    load_waypoints_service_ = this->create_service<nav2_navigation::srv::LoadWaypoints>(
        "/load_waypoints",
        std::bind(&GoalPublisher::loadWaypointsCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Timer for checking arrival
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period),
        std::bind(&GoalPublisher::timerCallback, this));

    // Initialize state
    is_navigating_ = false;
    navigation_mode_ = NavigationMode::OFF;
    is_forward_direction_ = true;
    going_to_start_ = false;
    current_waypoint_index_ = 0;
    has_robot_start_position_ = false;
    prev_waypoint_count_ = 0;

    RCLCPP_INFO(this->get_logger(), "GoalPublisher node initialized");
    RCLCPP_INFO(this->get_logger(), "  - Arrival threshold: %.2f m", arrival_threshold_);
    RCLCPP_INFO(this->get_logger(), "  - Navigation mode: OFF");
    RCLCPP_INFO(this->get_logger(), "  - Waiting for waypoints on /clicked_point");
    RCLCPP_INFO(this->get_logger(), "  - Call /auto_nav service to start navigation");
}

void GoalPublisher::loadWaypointsFromParams()
{
    auto waypoints_x = this->get_parameter("waypoints_x").as_double_array();
    auto waypoints_y = this->get_parameter("waypoints_y").as_double_array();

    if (waypoints_x.empty() || waypoints_y.empty()) {
        RCLCPP_INFO(this->get_logger(), "No waypoints in params, waiting for /clicked_point");
        return;
    }

    if (waypoints_x.size() != waypoints_y.size()) {
        RCLCPP_ERROR(this->get_logger(), "waypoints_x and waypoints_y size mismatch!");
        return;
    }

    for (size_t i = 0; i < waypoints_x.size(); i++) {
        geometry_msgs::msg::PointStamped point;
        point.header.frame_id = "map";
        point.header.stamp = this->now();
        point.point.x = waypoints_x[i];
        point.point.y = waypoints_y[i];
        point.point.z = 0.0;
        waypoints_.push_back(point);

        RCLCPP_INFO(this->get_logger(), "Loaded waypoint %zu from params: (%.2f, %.2f)",
                    i + 1, waypoints_x[i], waypoints_y[i]);
    }

    RCLCPP_INFO(this->get_logger(), "Total %zu waypoints loaded from params", waypoints_.size());
}

void GoalPublisher::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Save robot start position when adding first waypoint
    if (waypoints_.empty()) {
        updateRobotStartPosition();
    }

    waypoints_.push_back(*msg);
    RCLCPP_INFO(this->get_logger(), "Waypoint %zu added: (%.2f, %.2f, %.2f)",
                waypoints_.size(), msg->point.x, msg->point.y, msg->point.z);

    // Update markers
    publishMarkers();
}

void GoalPublisher::listWaypointsCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (waypoints_.empty()) {
        response->success = false;
        response->message = "No waypoints registered.";
        return;
    }

    std::string msg = "Waypoints (" + std::to_string(waypoints_.size()) + " total):\n";
    for (size_t i = 0; i < waypoints_.size(); i++) {
        const auto& wp = waypoints_[i];
        std::string marker = (i == current_waypoint_index_ && is_navigating_) ? " <-- current" : "";
        msg += "  [" + std::to_string(i + 1) + "] x=" + std::to_string(wp.point.x) +
               ", y=" + std::to_string(wp.point.y) + marker + "\n";
    }

    response->success = true;
    response->message = msg;
    RCLCPP_INFO(this->get_logger(), "\n%s", msg.c_str());
}

void GoalPublisher::removeLastWaypointCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (waypoints_.empty()) {
        response->success = false;
        response->message = "No waypoints to remove.";
        return;
    }

    if (is_navigating_) {
        response->success = false;
        response->message = "Cannot remove waypoint while navigating.";
        return;
    }

    auto removed = waypoints_.back();
    waypoints_.pop_back();

    response->success = true;
    response->message = "Removed waypoint: (" +
                       std::to_string(removed.point.x) + ", " +
                       std::to_string(removed.point.y) + "). " +
                       std::to_string(waypoints_.size()) + " waypoints remaining.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    // Update markers
    publishMarkers();
}

void GoalPublisher::clearAllWaypointsCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (waypoints_.empty()) {
        response->success = false;
        response->message = "No waypoints to clear.";
        return;
    }

    if (is_navigating_) {
        response->success = false;
        response->message = "Cannot clear waypoints while navigating.";
        return;
    }

    size_t count = waypoints_.size();
    waypoints_.clear();
    has_robot_start_position_ = false;  // Reset start position as well

    response->success = true;
    response->message = "Cleared all " + std::to_string(count) + " waypoints.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    // Update markers (delete all)
    publishMarkers();
}

void GoalPublisher::saveWaypointsCallback(
    const nav2_navigation::srv::SaveWaypoints::Request::SharedPtr request,
    nav2_navigation::srv::SaveWaypoints::Response::SharedPtr response)
{
    if (waypoints_.empty()) {
        response->success = false;
        response->message = "No waypoints to save.";
        return;
    }

    // Get file path from request, use default if empty
    std::string file_path = request->file_path.empty() ? waypoints_file_path_ : request->file_path;

    try {
        YAML::Node root;
        YAML::Node waypoints_node;

        for (size_t i = 0; i < waypoints_.size(); i++) {
            YAML::Node wp_node;
            wp_node["id"] = static_cast<int>(i + 1);
            wp_node["x"] = waypoints_[i].point.x;
            wp_node["y"] = waypoints_[i].point.y;
            waypoints_node.push_back(wp_node);
        }
        root["waypoints"] = waypoints_node;

        std::ofstream fout(file_path);
        if (!fout.is_open()) {
            response->success = false;
            response->message = "Failed to open file: " + file_path;
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        fout << root;
        fout.close();

        response->success = true;
        response->message = "Saved " + std::to_string(waypoints_.size()) +
                           " waypoints to " + file_path;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error saving waypoints: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void GoalPublisher::loadWaypointsCallback(
    const nav2_navigation::srv::LoadWaypoints::Request::SharedPtr request,
    nav2_navigation::srv::LoadWaypoints::Response::SharedPtr response)
{
    if (is_navigating_) {
        response->success = false;
        response->message = "Cannot load waypoints while navigating.";
        return;
    }

    // Get file path from request, use default if empty
    std::string file_path = request->file_path.empty() ? waypoints_file_path_ : request->file_path;

    try {
        YAML::Node root = YAML::LoadFile(file_path);

        if (!root["waypoints"]) {
            response->success = false;
            response->message = "Invalid YAML format: 'waypoints' key not found.";
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Clear existing waypoints
        waypoints_.clear();
        has_robot_start_position_ = false;

        // Save robot current position (for markers)
        updateRobotStartPosition();

        YAML::Node waypoints_node = root["waypoints"];
        int index = 1;
        for (const auto& wp_node : waypoints_node) {
            // Check if x, y fields exist
            if (!wp_node["x"] || !wp_node["y"]) {
                RCLCPP_WARN(this->get_logger(), "Waypoint %d missing x or y field, skipping", index);
                index++;
                continue;
            }

            geometry_msgs::msg::PointStamped point;
            point.header.frame_id = "map";
            point.header.stamp = this->now();
            point.point.x = wp_node["x"].as<double>();
            point.point.y = wp_node["y"].as<double>();
            point.point.z = 0.0;
            waypoints_.push_back(point);

            // id field is optional (use index if not present)
            int id = wp_node["id"] ? wp_node["id"].as<int>() : index;
            RCLCPP_INFO(this->get_logger(), "Loaded waypoint %d: (%.2f, %.2f)",
                        id, point.point.x, point.point.y);
            index++;
        }

        // Update markers
        publishMarkers();

        response->success = true;
        response->message = "Loaded " + std::to_string(waypoints_.size()) +
                           " waypoints from " + file_path;
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    } catch (const YAML::BadFile& e) {
        response->success = false;
        response->message = "File not found: " + file_path;
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = "Error loading waypoints: " + std::string(e.what());
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
    }
}

void GoalPublisher::autoNavCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (waypoints_.empty()) {
        response->success = false;
        response->message = "No waypoints available. Add waypoints using /clicked_point first.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    if (is_navigating_) {
        response->success = false;
        response->message = "Already navigating. Wait for completion or restart the node.";
        RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        return;
    }

    // Save start position (for loop mode)
    updateRobotStartPosition();

    // Start navigation from the first waypoint
    is_navigating_ = true;
    is_forward_direction_ = true;
    going_to_start_ = false;
    current_waypoint_index_ = 0;

    publishCurrentGoal();

    response->success = true;
    std::string mode_status;
    switch (navigation_mode_) {
        case NavigationMode::LOOP:
            mode_status = " (Loop mode)";
            break;
        case NavigationMode::ROUND_TRIP:
            mode_status = " (Round Trip mode)";
            break;
        default:
            mode_status = "";
            break;
    }
    response->message = "Auto navigation started with " + std::to_string(waypoints_.size()) + " waypoints." + mode_status;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void GoalPublisher::stopNavCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    if (!is_navigating_) {
        response->success = false;
        response->message = "Not currently navigating.";
        return;
    }

    is_navigating_ = false;
    is_forward_direction_ = true;
    going_to_start_ = false;
    current_waypoint_index_ = 0;

    response->success = true;
    response->message = "Navigation stopped.";
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());

    // Update markers (delete current_goal)
    publishMarkers();
}

void GoalPublisher::toggleLoopModeCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response)
{
    // Cycle through: OFF -> LOOP -> ROUND_TRIP -> OFF
    switch (navigation_mode_) {
        case NavigationMode::OFF:
            navigation_mode_ = NavigationMode::LOOP;
            response->message = "Mode: LOOP";
            break;
        case NavigationMode::LOOP:
            navigation_mode_ = NavigationMode::ROUND_TRIP;
            response->message = "Mode: ROUND_TRIP";
            break;
        case NavigationMode::ROUND_TRIP:
            navigation_mode_ = NavigationMode::OFF;
            response->message = "Mode: OFF";
            break;
    }

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

void GoalPublisher::timerCallback()
{
    // Publish markers periodically (if waypoints exist)
    if (!waypoints_.empty()) {
        publishMarkers();
    }

    if (!is_navigating_ || waypoints_.empty()) {
        return;
    }

    // Get robot's current position from TF
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_DEBUG(this->get_logger(), "Could not get transform: %s", ex.what());
        return;
    }

    // Determine current target position
    double target_x, target_y;
    if (going_to_start_) {
        target_x = robot_start_position_.x;
        target_y = robot_start_position_.y;
    } else {
        target_x = waypoints_[current_waypoint_index_].point.x;
        target_y = waypoints_[current_waypoint_index_].point.y;
    }

    double dx = target_x - transform.transform.translation.x;
    double dy = target_y - transform.transform.translation.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Check arrival
    if (distance < arrival_threshold_) {
        if (going_to_start_) {
            // Arrived at start position
            RCLCPP_INFO(this->get_logger(), "Arrived at start position");

            if (navigation_mode_ == NavigationMode::ROUND_TRIP) {
                // Round Trip mode: switch to forward direction, go to first waypoint
                is_forward_direction_ = true;
                going_to_start_ = false;
                current_waypoint_index_ = 0;
                publishCurrentGoal();
            } else {
                // If OFF mode, complete here (this case is generally rare)
                is_navigating_ = false;
                RCLCPP_INFO(this->get_logger(), "Navigation completed!");
                publishMarkers();
            }
        } else {
            // Arrived at waypoint
            RCLCPP_INFO(this->get_logger(), "Arrived at waypoint %zu/%zu",
                        current_waypoint_index_ + 1, waypoints_.size());

            if (is_forward_direction_) {
                // Moving forward
                if (current_waypoint_index_ + 1 < waypoints_.size()) {
                    // Go to next waypoint
                    current_waypoint_index_++;
                    publishCurrentGoal();
                } else {
                    // Arrived at last waypoint
                    if (navigation_mode_ == NavigationMode::LOOP) {
                        // Loop mode: return directly to first waypoint
                        current_waypoint_index_ = 0;
                        RCLCPP_INFO(this->get_logger(), "Loop mode: Going back to first waypoint");
                        publishCurrentGoal();
                    } else if (navigation_mode_ == NavigationMode::ROUND_TRIP) {
                        // Round Trip mode: switch to reverse direction
                        is_forward_direction_ = false;
                        RCLCPP_INFO(this->get_logger(), "Round Trip mode: Reversing direction");
                        if (waypoints_.size() > 1) {
                            current_waypoint_index_--;
                            publishCurrentGoal();
                        } else {
                            // If only one waypoint, go directly to start position
                            going_to_start_ = true;
                            publishCurrentGoal();
                        }
                    } else {
                        // If OFF mode, complete navigation
                        is_navigating_ = false;
                        RCLCPP_INFO(this->get_logger(), "All waypoints completed!");
                        publishMarkers();
                    }
                }
            } else {
                // Moving backward (Round Trip mode)
                if (current_waypoint_index_ > 0) {
                    // Go to previous waypoint
                    current_waypoint_index_--;
                    publishCurrentGoal();
                } else {
                    // Arrived at first waypoint, go to start position
                    going_to_start_ = true;
                    RCLCPP_INFO(this->get_logger(), "Round Trip mode: Returning to start position");
                    publishCurrentGoal();
                }
            }
        }
    }
}

void GoalPublisher::publishCurrentGoal()
{
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";

    double goal_x, goal_y;
    double yaw = 0.0;

    if (going_to_start_) {
        // Move to start position
        goal_x = robot_start_position_.x;
        goal_y = robot_start_position_.y;

        // Direction facing start position (from waypoint 0 to start position)
        if (!waypoints_.empty()) {
            double dx = goal_x - waypoints_[0].point.x;
            double dy = goal_y - waypoints_[0].point.y;
            yaw = std::atan2(dy, dx);
        }

        RCLCPP_INFO(this->get_logger(), "Published goal: Start position (%.2f, %.2f)",
                    goal_x, goal_y);
    } else {
        // Move to waypoint
        if (current_waypoint_index_ >= waypoints_.size()) {
            return;
        }

        const auto& waypoint = waypoints_[current_waypoint_index_];
        goal_x = waypoint.point.x;
        goal_y = waypoint.point.y;

        // Calculate yaw based on movement direction
        if (is_forward_direction_) {
            // Forward: face next waypoint
            if (current_waypoint_index_ + 1 < waypoints_.size()) {
                const auto& next_wp = waypoints_[current_waypoint_index_ + 1];
                double dx = next_wp.point.x - goal_x;
                double dy = next_wp.point.y - goal_y;
                yaw = std::atan2(dy, dx);
            }
        } else {
            // Backward: face previous waypoint or start position
            if (current_waypoint_index_ > 0) {
                const auto& prev_wp = waypoints_[current_waypoint_index_ - 1];
                double dx = prev_wp.point.x - goal_x;
                double dy = prev_wp.point.y - goal_y;
                yaw = std::atan2(dy, dx);
            } else {
                // If first waypoint, face start position
                double dx = robot_start_position_.x - goal_x;
                double dy = robot_start_position_.y - goal_y;
                yaw = std::atan2(dy, dx);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Published goal %zu/%zu: (%.2f, %.2f) [%s]",
                    current_waypoint_index_ + 1, waypoints_.size(),
                    goal_x, goal_y,
                    is_forward_direction_ ? "forward" : "backward");
    }

    goal_pose.pose.position.x = goal_x;
    goal_pose.pose.position.y = goal_y;
    goal_pose.pose.position.z = 0.0;

    // Yaw to Quaternion conversion
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    goal_pose.pose.orientation.x = q.x();
    goal_pose.pose.orientation.y = q.y();
    goal_pose.pose.orientation.z = q.z();
    goal_pose.pose.orientation.w = q.w();

    // Publish multiple times to ensure Nav2 receives it
    for (int i = 0; i < 5; i++) {
        goal_pose.header.stamp = this->now();  // Update timestamp each time
        goal_pose_pub_->publish(goal_pose);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void GoalPublisher::publishMarkers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Delete all markers if waypoints is empty
    if (waypoints_.empty()) {
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = "map";
        delete_marker.header.stamp = this->now();
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);
        marker_pub_->publish(marker_array);
        return;
    }

    // 1. Waypoint spheres (points)
    visualization_msgs::msg::Marker sphere_marker;
    sphere_marker.header.frame_id = "map";
    sphere_marker.header.stamp = this->now();
    sphere_marker.ns = "waypoints";
    sphere_marker.id = 0;
    sphere_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere_marker.action = visualization_msgs::msg::Marker::ADD;
    sphere_marker.scale.x = 0.3;  // sphere diameter
    sphere_marker.scale.y = 0.3;
    sphere_marker.scale.z = 0.3;
    sphere_marker.color.r = 0.0;
    sphere_marker.color.g = 1.0;
    sphere_marker.color.b = 0.0;
    sphere_marker.color.a = 1.0;
    sphere_marker.pose.orientation.w = 1.0;

    for (const auto& wp : waypoints_) {
        geometry_msgs::msg::Point p;
        p.x = wp.point.x;
        p.y = wp.point.y;
        p.z = 0.1;  // slightly above ground
        sphere_marker.points.push_back(p);
    }
    marker_array.markers.push_back(sphere_marker);

    // 2. Path line (robot start position to waypoints connection)
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->now();
    line_marker.ns = "path";
    line_marker.id = 1;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;  // line width
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.5;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.8;
    line_marker.pose.orientation.w = 1.0;

    // Add robot start position (if exists)
    if (has_robot_start_position_) {
        geometry_msgs::msg::Point start_p;
        start_p.x = robot_start_position_.x;
        start_p.y = robot_start_position_.y;
        start_p.z = 0.05;
        line_marker.points.push_back(start_p);
    }

    // Add waypoints
    for (const auto& wp : waypoints_) {
        geometry_msgs::msg::Point p;
        p.x = wp.point.x;
        p.y = wp.point.y;
        p.z = 0.05;
        line_marker.points.push_back(p);
    }
    marker_array.markers.push_back(line_marker);

    // 3. Waypoint numbers (text labels)
    for (size_t i = 0; i < waypoints_.size(); i++) {
        visualization_msgs::msg::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = this->now();
        text_marker.ns = "waypoint_labels";
        text_marker.id = static_cast<int>(i + 10);  // unique ID
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        text_marker.pose.position.x = waypoints_[i].point.x;
        text_marker.pose.position.y = waypoints_[i].point.y;
        text_marker.pose.position.z = 0.5;  // above sphere
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.3;  // text height
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;
        text_marker.text = std::to_string(i + 1);
        marker_array.markers.push_back(text_marker);
    }

    // Delete remaining label markers if waypoints decreased
    if (prev_waypoint_count_ > waypoints_.size()) {
        for (size_t i = waypoints_.size(); i < prev_waypoint_count_; i++) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = "map";
            delete_marker.header.stamp = this->now();
            delete_marker.ns = "waypoint_labels";
            delete_marker.id = static_cast<int>(i + 10);
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
        }
    }
    prev_waypoint_count_ = waypoints_.size();

    // 4. Current goal highlight (when navigating)
    if (is_navigating_ && current_waypoint_index_ < waypoints_.size()) {
        visualization_msgs::msg::Marker current_marker;
        current_marker.header.frame_id = "map";
        current_marker.header.stamp = this->now();
        current_marker.ns = "current_goal";
        current_marker.id = 100;
        current_marker.type = visualization_msgs::msg::Marker::SPHERE;
        current_marker.action = visualization_msgs::msg::Marker::ADD;
        current_marker.pose.position.x = waypoints_[current_waypoint_index_].point.x;
        current_marker.pose.position.y = waypoints_[current_waypoint_index_].point.y;
        current_marker.pose.position.z = 0.1;
        current_marker.pose.orientation.w = 1.0;
        current_marker.scale.x = 0.5;
        current_marker.scale.y = 0.5;
        current_marker.scale.z = 0.5;
        current_marker.color.r = 1.0;
        current_marker.color.g = 0.0;
        current_marker.color.b = 0.0;
        current_marker.color.a = 0.7;
        marker_array.markers.push_back(current_marker);
    }

    marker_pub_->publish(marker_array);
}

void GoalPublisher::updateRobotStartPosition()
{
    if (has_robot_start_position_) {
        return;  // already have start position
    }

    try {
        auto transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        robot_start_position_.x = transform.transform.translation.x;
        robot_start_position_.y = transform.transform.translation.y;
        robot_start_position_.z = 0.0;
        has_robot_start_position_ = true;
        RCLCPP_INFO(this->get_logger(), "Robot start position saved: (%.2f, %.2f)",
                    robot_start_position_.x, robot_start_position_.y);
    } catch (tf2::TransformException &ex) {
        RCLCPP_DEBUG(this->get_logger(), "Could not get robot position: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoalPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}