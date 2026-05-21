#include "nav2_navigation/goal_publisher_gui.hpp"

#include <QApplication>
#include <QMessageBox>
#include <QDir>

GoalPublisherGui::GoalPublisherGui(std::shared_ptr<rclcpp::Node> node, QWidget* parent)
    : QWidget(parent), node_(node), current_mode_(GuiNavigationMode::OFF)
{
    setupUi();
    setupRosClients();

    // Setup ROS2 spin timer
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &GoalPublisherGui::spinOnce);
    ros_timer_->start(50);  // 50ms = 20Hz

    appendStatus("GUI initialized. Waiting for goal_publisher node...");
}

void GoalPublisherGui::setupUi()
{
    setWindowTitle("Goal Publisher Control");
    setMinimumSize(400, 300);

    // Main layout
    QVBoxLayout* main_layout = new QVBoxLayout(this);

    // Title
    title_label_ = new QLabel("Goal Publisher Control Panel");
    title_label_->setStyleSheet("font-size: 16px; font-weight: bold; margin-bottom: 10px;");
    title_label_->setAlignment(Qt::AlignCenter);
    main_layout->addWidget(title_label_);

    // Button layout
    QHBoxLayout* button_layout = new QHBoxLayout();

    // Start Navigation button
    start_nav_btn_ = new QPushButton("Start Nav");
    start_nav_btn_->setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;");
    start_nav_btn_->setMinimumHeight(40);
    connect(start_nav_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onStartNavClicked);
    button_layout->addWidget(start_nav_btn_);

    // Stop Navigation button
    stop_nav_btn_ = new QPushButton("Stop Nav");
    stop_nav_btn_->setStyleSheet("background-color: #E91E63; color: white; padding: 10px;");
    stop_nav_btn_->setMinimumHeight(40);
    connect(stop_nav_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onStopNavClicked);
    button_layout->addWidget(stop_nav_btn_);

    // Mode toggle button (OFF -> Loop -> Round Trip -> OFF)
    loop_mode_btn_ = new QPushButton("Mode: OFF");
    loop_mode_btn_->setStyleSheet("background-color: #795548; color: white; padding: 10px;");
    loop_mode_btn_->setMinimumHeight(40);
    connect(loop_mode_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onToggleLoopModeClicked);
    button_layout->addWidget(loop_mode_btn_);

    main_layout->addLayout(button_layout);

    // Second row buttons
    QHBoxLayout* button_layout1_5 = new QHBoxLayout();

    // List Waypoints button
    list_wp_btn_ = new QPushButton("List Waypoints");
    list_wp_btn_->setStyleSheet("background-color: #2196F3; color: white; padding: 10px;");
    list_wp_btn_->setMinimumHeight(40);
    connect(list_wp_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onListWaypointsClicked);
    button_layout1_5->addWidget(list_wp_btn_);

    main_layout->addLayout(button_layout1_5);

    // Second row buttons
    QHBoxLayout* button_layout2 = new QHBoxLayout();

    // Remove Last Waypoint button
    remove_last_btn_ = new QPushButton("Remove Last WP");
    remove_last_btn_->setStyleSheet("background-color: #FF9800; color: white; padding: 10px;");
    remove_last_btn_->setMinimumHeight(40);
    connect(remove_last_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onRemoveLastClicked);
    button_layout2->addWidget(remove_last_btn_);

    // Clear All Waypoints button
    clear_wp_btn_ = new QPushButton("Clear All WP");
    clear_wp_btn_->setStyleSheet("background-color: #f44336; color: white; padding: 10px;");
    clear_wp_btn_->setMinimumHeight(40);
    connect(clear_wp_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onClearWaypointsClicked);
    button_layout2->addWidget(clear_wp_btn_);

    main_layout->addLayout(button_layout2);

    // Third row buttons (Save/Load)
    QHBoxLayout* button_layout3 = new QHBoxLayout();

    // Save Waypoints button
    save_wp_btn_ = new QPushButton("Save WP");
    save_wp_btn_->setStyleSheet("background-color: #9C27B0; color: white; padding: 10px;");
    save_wp_btn_->setMinimumHeight(40);
    connect(save_wp_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onSaveWaypointsClicked);
    button_layout3->addWidget(save_wp_btn_);

    // Load Waypoints button
    load_wp_btn_ = new QPushButton("Load WP");
    load_wp_btn_->setStyleSheet("background-color: #607D8B; color: white; padding: 10px;");
    load_wp_btn_->setMinimumHeight(40);
    connect(load_wp_btn_, &QPushButton::clicked, this, &GoalPublisherGui::onLoadWaypointsClicked);
    button_layout3->addWidget(load_wp_btn_);

    main_layout->addLayout(button_layout3);

    // Status display
    QLabel* status_label = new QLabel("Status:");
    main_layout->addWidget(status_label);

    status_display_ = new QTextEdit();
    status_display_->setReadOnly(true);
    status_display_->setStyleSheet("background-color: #f5f5f5; font-family: monospace;");
    main_layout->addWidget(status_display_);

    setLayout(main_layout);
}

void GoalPublisherGui::setupRosClients()
{
    auto_nav_client_ = node_->create_client<std_srvs::srv::Trigger>("/auto_nav");
    stop_nav_client_ = node_->create_client<std_srvs::srv::Trigger>("/stop_nav");
    toggle_loop_client_ = node_->create_client<std_srvs::srv::Trigger>("/toggle_loop_mode");
    list_wp_client_ = node_->create_client<std_srvs::srv::Trigger>("/list_waypoints");
    remove_wp_client_ = node_->create_client<std_srvs::srv::Trigger>("/remove_last_waypoint");
    clear_wp_client_ = node_->create_client<std_srvs::srv::Trigger>("/clear_all_waypoints");
    save_wp_client_ = node_->create_client<nav2_navigation::srv::SaveWaypoints>("/save_waypoints");
    load_wp_client_ = node_->create_client<nav2_navigation::srv::LoadWaypoints>("/load_waypoints");
}

void GoalPublisherGui::appendStatus(const std::string& message, bool is_error)
{
    QString color = is_error ? "red" : "black";
    QString html = QString("<span style='color:%1;'>%2</span><br>")
                       .arg(color)
                       .arg(QString::fromStdString(message));
    status_display_->insertHtml(html);
    status_display_->ensureCursorVisible();
}

void GoalPublisherGui::spinOnce()
{
    if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }
}

void GoalPublisherGui::onStartNavClicked()
{
    if (!auto_nav_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /auto_nav not available!", true);
        return;
    }

    appendStatus("Starting navigation...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = auto_nav_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus("Failed: " + result->message, true);
            }
        });
}

void GoalPublisherGui::onStopNavClicked()
{
    if (!stop_nav_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /stop_nav not available!", true);
        return;
    }

    appendStatus("Stopping navigation...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = stop_nav_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::onToggleLoopModeClicked()
{
    if (!toggle_loop_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /toggle_loop_mode not available!", true);
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = toggle_loop_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
                // Update button state (OFF -> LOOP -> ROUND_TRIP -> OFF)
                if (result->message.find("LOOP") != std::string::npos &&
                    result->message.find("ROUND_TRIP") == std::string::npos) {
                    current_mode_ = GuiNavigationMode::LOOP;
                } else if (result->message.find("ROUND_TRIP") != std::string::npos) {
                    current_mode_ = GuiNavigationMode::ROUND_TRIP;
                } else {
                    current_mode_ = GuiNavigationMode::OFF;
                }
                updateModeButton();
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::updateModeButton()
{
    switch (current_mode_) {
        case GuiNavigationMode::OFF:
            loop_mode_btn_->setText("Mode: OFF");
            loop_mode_btn_->setStyleSheet("background-color: #795548; color: white; padding: 10px;");
            break;
        case GuiNavigationMode::LOOP:
            loop_mode_btn_->setText("Mode: LOOP");
            loop_mode_btn_->setStyleSheet("background-color: #2196F3; color: white; padding: 10px;");
            break;
        case GuiNavigationMode::ROUND_TRIP:
            loop_mode_btn_->setText("Mode: R-TRIP");
            loop_mode_btn_->setStyleSheet("background-color: #4CAF50; color: white; padding: 10px;");
            break;
    }
}

void GoalPublisherGui::onListWaypointsClicked()
{
    if (!list_wp_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /list_waypoints not available!", true);
        return;
    }

    appendStatus("Fetching waypoints...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = list_wp_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::onRemoveLastClicked()
{
    if (!remove_wp_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /remove_last_waypoint not available!", true);
        return;
    }

    appendStatus("Removing last waypoint...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = remove_wp_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::onClearWaypointsClicked()
{
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Confirm",
                                  "Clear all waypoints?",
                                  QMessageBox::Yes | QMessageBox::No);

    if (reply != QMessageBox::Yes) {
        return;
    }

    if (!clear_wp_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /clear_all_waypoints not available!", true);
        return;
    }

    appendStatus("Clearing all waypoints...");

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = clear_wp_client_->async_send_request(request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::onSaveWaypointsClicked()
{
     // Open file save dialog
    QString file_path = QFileDialog::getSaveFileName(
        this,
        "Save Waypoints",
        QDir::homePath() + "/waypoints.yaml",
        "YAML Files (*.yaml *.yml);;All Files (*)"
    );

    if (file_path.isEmpty()) {
        appendStatus("Save cancelled.");
        return;
    }

    // Check service availability
    if (!save_wp_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /save_waypoints not available!", true);
        return;
    }

    appendStatus("Saving waypoints to: " + file_path.toStdString());

    auto request = std::make_shared<nav2_navigation::srv::SaveWaypoints::Request>();
    request->file_path = file_path.toStdString();

    auto future = save_wp_client_->async_send_request(request,
        [this](rclcpp::Client<nav2_navigation::srv::SaveWaypoints>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

void GoalPublisherGui::onLoadWaypointsClicked()
{
    // Open file save dialog
    QString file_path = QFileDialog::getOpenFileName(
        this,
        "Load Waypoints",
        QDir::homePath(),
        "YAML Files (*.yaml *.yml);;All Files (*)"
    );

    if (file_path.isEmpty()) {
        appendStatus("Load cancelled.");
        return;
    }

    // Check service availability
    if (!load_wp_client_->wait_for_service(std::chrono::seconds(1))) {
        appendStatus("Service /load_waypoints not available!", true);
        return;
    }

    appendStatus("Loading waypoints from: " + file_path.toStdString());

    auto request = std::make_shared<nav2_navigation::srv::LoadWaypoints::Request>();
    request->file_path = file_path.toStdString();

    auto future = load_wp_client_->async_send_request(request,
        [this](rclcpp::Client<nav2_navigation::srv::LoadWaypoints>::SharedFuture response) {
            auto result = response.get();
            if (result->success) {
                appendStatus(result->message);
            } else {
                appendStatus(result->message, true);
            }
        });
}

int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create ROS2 node
    auto node = std::make_shared<rclcpp::Node>("goal_publisher_gui");

    // Initialize Qt
    QApplication app(argc, argv);

    // Create and show GUI
    GoalPublisherGui gui(node);
    gui.show();

    // Run Qt event loop
    int result = app.exec();

    // Cleanup
    rclcpp::shutdown();
    return result;
}
