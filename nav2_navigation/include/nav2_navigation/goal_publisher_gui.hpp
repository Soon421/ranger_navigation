#ifndef NAV2_NAVIGATION__GOAL_PUBLISHER_GUI_HPP_
#define NAV2_NAVIGATION__GOAL_PUBLISHER_GUI_HPP_

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QTimer>
#include <QFileDialog>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "nav2_navigation/srv/save_waypoints.hpp"
#include "nav2_navigation/srv/load_waypoints.hpp"

#include <memory>
#include <string>

// Navigation mode enum (must match goal_publisher.hpp)
enum class GuiNavigationMode {
    OFF,
    LOOP,
    ROUND_TRIP
};

class GoalPublisherGui : public QWidget
{
    Q_OBJECT

public:
    explicit GoalPublisherGui(std::shared_ptr<rclcpp::Node> node, QWidget* parent = nullptr);
    ~GoalPublisherGui() = default;

private slots:
    void onStartNavClicked();
    void onStopNavClicked();
    void onToggleLoopModeClicked();
    void onListWaypointsClicked();
    void onRemoveLastClicked();
    void onClearWaypointsClicked();
    void onSaveWaypointsClicked();
    void onLoadWaypointsClicked();
    void spinOnce();

private:
    void setupUi();
    void setupRosClients();
    void appendStatus(const std::string& message, bool is_error = false);

    // ROS2 node
    std::shared_ptr<rclcpp::Node> node_;

    // Qt widgets
    QPushButton* start_nav_btn_;
    QPushButton* stop_nav_btn_;
    QPushButton* loop_mode_btn_;
    QPushButton* list_wp_btn_;
    QPushButton* remove_last_btn_;
    QPushButton* clear_wp_btn_;
    QPushButton* save_wp_btn_;
    QPushButton* load_wp_btn_;
    QTextEdit* status_display_;
    QLabel* title_label_;

    // Qt timer for ROS2 spinning
    QTimer* ros_timer_;

    // ROS2 service clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr auto_nav_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_nav_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_loop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr list_wp_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr remove_wp_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_wp_client_;
    rclcpp::Client<nav2_navigation::srv::SaveWaypoints>::SharedPtr save_wp_client_;
    rclcpp::Client<nav2_navigation::srv::LoadWaypoints>::SharedPtr load_wp_client_;

    // Navigation mode state for GUI
    GuiNavigationMode current_mode_;
    void updateModeButton();
};

#endif  // NAV2_NAVIGATION__GOAL_PUBLISHER_GUI_HPP_
