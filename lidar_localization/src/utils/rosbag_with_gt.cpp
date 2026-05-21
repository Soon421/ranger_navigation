#include "rosbag_with_gt.hpp"

RosbagWithGT::RosbagWithGT(const std::string& ground_truth_topic, const std::string& odometry_topic)
    : groundtruth_topic_(ground_truth_topic),
      odometry_topic_(odometry_topic),
      modified_odometry_topic_("/ground_truth") {}

RosbagWithGT::~RosbagWithGT() {}

bool RosbagWithGT::readRosbag(const std::string& input_bag_path) {
    try {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = input_bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        reader.open(storage_options, converter_options);

        // Get topic metadata (including QoS profiles)
        auto topics = reader.get_all_topics_and_types();
        for (const auto& topic : topics) {
            topic_types_[topic.name] = topic.type;
            topic_metadata_list_.push_back(topic);
        }

        std::cout << "Reading bag file: " << input_bag_path << std::endl;

        // Read all messages
        while (reader.has_next()) {
            auto bag_message = reader.read_next();

            BagMessage msg;
            msg.topic = bag_message->topic_name;
            msg.type = topic_types_[bag_message->topic_name];
            msg.timestamp = rclcpp::Time(bag_message->time_stamp);
            msg.serialized_msg = std::make_shared<rclcpp::SerializedMessage>(*bag_message->serialized_data);

            messages_.push_back(std::move(msg));
        }

        std::cout << "Read " << messages_.size() << " messages from bag." << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error opening file: " << input_bag_path << " - " << e.what() << std::endl;
        return false;
    }
}

nav_msgs::msg::Path RosbagWithGT::getGroundtruthPath() {
    nav_msgs::msg::Path last_path;

    for (const auto& msg : messages_) {
        if (msg.topic == groundtruth_topic_ && msg.type == "nav_msgs/msg/Path") {
            nav_msgs::msg::Path path;
            path_serializer_.deserialize_message(msg.serialized_msg.get(), &path);
            last_path = path;
        }
    }

    return last_path;
}

bool RosbagWithGT::writeRosbag(const std::string& output_bag_path) {
    std::cout << "Start to write rosbag file!" << std::endl;

    try {
        rosbag2_cpp::Writer writer;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = output_bag_path;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        writer.open(storage_options, converter_options);

        nav_msgs::msg::Path groundtruth_path = getGroundtruthPath();
        if (groundtruth_path.poses.empty()) {
            std::cerr << "Ground truth path does not exist!" << std::endl;
            return false;
        }

        int groundtruth_index = -1;
        nav_msgs::msg::Odometry groundtruth_odom, prev_odom, modified_odom;
        Eigen::Matrix4f modified_odom_matrix, pre_relative_odom_matrix;
        prev_odom = matrix2odom(Eigen::Matrix4f::Identity());
        pre_relative_odom_matrix = Eigen::Matrix4f::Identity();
        bool initial_gt_odom_assigned = false;

        int count = 0;
        bool odometry_topic_exist = false;

        for (const auto& msg : messages_) {
            progressBar(count, messages_.size());

            if (msg.topic == odometry_topic_) {
                odometry_topic_exist = true;

                // Deserialize odometry message
                nav_msgs::msg::Odometry current_odom;
                odom_serializer_.deserialize_message(msg.serialized_msg.get(), &current_odom);

                auto& msg_stamp = current_odom.header.stamp;
                double msg_time = static_cast<double>(msg_stamp.sec) + static_cast<double>(msg_stamp.nanosec) * 1e-9;

                // 인덱스 범위 체크
                int next_gt_index = groundtruth_index + 1;
                if (next_gt_index >= static_cast<int>(groundtruth_path.poses.size())) {
                    // ground truth 범위를 벗어나면 원본 메시지만 기록하고 스킵
                    writer.write(msg.serialized_msg, msg.topic, msg.type, msg.timestamp);
                    count++;
                    continue;
                }

                double gt_time = rclcpp::Time(groundtruth_path.poses[next_gt_index].header.stamp).seconds();

                if (gt_time < msg_time &&
                    next_gt_index < static_cast<int>(groundtruth_path.poses.size())) {
                    groundtruth_odom.pose.pose = groundtruth_path.poses[next_gt_index].pose;
                    modified_odom_matrix = odom2matrix(groundtruth_odom);
                    initial_gt_odom_assigned = true;
                    groundtruth_index++;
                }

                if (initial_gt_odom_assigned) {
                    // 타임스탬프가 유효한지 체크 (음수 타임스탬프 방지)
                    if (msg.timestamp.nanoseconds() <= 0) {
                        std::cerr << "Warning: Invalid timestamp detected, skipping message" << std::endl;
                        writer.write(msg.serialized_msg, msg.topic, msg.type, msg.timestamp);
                        count++;
                        continue;
                    }

                    Eigen::Matrix4f relative_odom_matrix = odom2matrix(prev_odom).inverse() * odom2matrix(current_odom);

                    if (odomDistance(prev_odom, current_odom) > 0.05 ||
                        odomHeadingAngleDifference(prev_odom, current_odom) > 0.2) {
                        relative_odom_matrix = pre_relative_odom_matrix;
                    }

                    modified_odom_matrix = modified_odom_matrix * relative_odom_matrix;
                    modified_odom = matrix2odom(modified_odom_matrix);

                    modified_odom.header = current_odom.header;
                    modified_odom.child_frame_id = current_odom.child_frame_id;

                    // Write modified odometry
                    writer.write(modified_odom, modified_odometry_topic_, msg.timestamp);

                    // Save TF
                    geometry_msgs::msg::TransformStamped tf_stamped;
                    tf_stamped.header.stamp = current_odom.header.stamp;
                    tf_stamped.header.frame_id = "odom";
                    tf_stamped.child_frame_id = "velodyne";
                    tf_stamped.transform.translation.x = modified_odom.pose.pose.position.x;
                    tf_stamped.transform.translation.y = modified_odom.pose.pose.position.y;
                    tf_stamped.transform.translation.z = modified_odom.pose.pose.position.z;
                    tf_stamped.transform.rotation = modified_odom.pose.pose.orientation;

                    tf2_msgs::msg::TFMessage tf_msg;
                    tf_msg.transforms.push_back(tf_stamped);

                    writer.write(tf_msg, "/tf", msg.timestamp);

                    prev_odom = current_odom;
                    pre_relative_odom_matrix = relative_odom_matrix;
                }

                // Write original odometry
                writer.write(msg.serialized_msg, msg.topic, msg.type, msg.timestamp);

            } else {
                // Write all other messages
                writer.write(msg.serialized_msg, msg.topic, msg.type, msg.timestamp);
            }
            count++;
        }

        if (!odometry_topic_exist) {
            std::cout << std::endl;
            std::cerr << "Warning: Odometry topic does not exist!" << std::endl;
        }

        std::cout << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error writing bag: " << e.what() << std::endl;
        return false;
    }
}

double RosbagWithGT::odomDistance(const nav_msgs::msg::Odometry& odom_1, const nav_msgs::msg::Odometry& odom_2) {
    return sqrt(pow(odom_1.pose.pose.position.x - odom_2.pose.pose.position.x, 2) +
                pow(odom_1.pose.pose.position.y - odom_2.pose.pose.position.y, 2) +
                pow(odom_1.pose.pose.position.z - odom_2.pose.pose.position.z, 2));
}

double RosbagWithGT::odomHeadingAngleDifference(const nav_msgs::msg::Odometry& odom_1, const nav_msgs::msg::Odometry& odom_2) {
    tf2::Quaternion q1, q2;
    tf2::fromMsg(odom_1.pose.pose.orientation, q1);
    tf2::fromMsg(odom_2.pose.pose.orientation, q2);

    tf2::Matrix3x3 m1(q1);
    tf2::Matrix3x3 m2(q2);

    tf2::Vector3 x1 = m1.getColumn(0);
    tf2::Vector3 x2 = m2.getColumn(0);

    x1.normalize();
    x2.normalize();

    double dot_product = x1.dot(x2);
    return acos(std::max(-1.0, std::min(1.0, dot_product)));
}

Eigen::Matrix4f RosbagWithGT::odom2matrix(const nav_msgs::msg::Odometry& odom) {
    double x, y, z, roll, pitch, yaw;
    x = odom.pose.pose.position.x;
    y = odom.pose.pose.position.y;
    z = odom.pose.pose.position.z;

    tf2::Quaternion orientation;
    tf2::fromMsg(odom.pose.pose.orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    return pcl::getTransformation(x, y, z, roll, pitch, yaw).matrix();
}

nav_msgs::msg::Odometry RosbagWithGT::matrix2odom(const Eigen::Matrix4f& matrix) {
    Eigen::Quaternionf orientation(matrix.block<3, 3>(0, 0));
    Eigen::Vector3f position(matrix.block<3, 1>(0, 3));

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    return odom_msg;
}

void RosbagWithGT::progressBar(const int index, const int max) {
    // 1% 단위로만 출력
    int current_percent = static_cast<int>(static_cast<float>(index) / max * 100);
    static int last_percent = -1;

    if (current_percent == last_percent && index != max - 1) {
        return;
    }
    last_percent = current_percent;

    std::cout << "Processing: " << index << " / " << max << " (" << current_percent << "%)" << std::endl;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if (argc < 3 || argc > 5) {
        std::cerr << "Usage: rosbag_with_gt input_bag output_bag [ground_truth_topic] [odometry_topic]" << std::endl;
        std::cerr << "  input_bag: Path to input ROS2 bag directory" << std::endl;
        std::cerr << "  output_bag: Path to output ROS2 bag directory" << std::endl;
        std::cerr << "  ground_truth_topic: (optional) Topic name for ground truth path (default: /lidar_localization/mapping/path)" << std::endl;
        std::cerr << "  odometry_topic: (optional) Topic name for odometry (default: /odometry/imu)" << std::endl;
        return 1;
    }

    std::string input_bag_path = argv[1];
    std::string output_bag_path = argv[2];

    if (input_bag_path == output_bag_path) {
        std::cerr << "Error: Input and output bag paths cannot be the same." << std::endl;
        return 1;
    }

    std::string ground_truth_topic = (argc >= 4) ? argv[3] : "/lidar_localization/mapping/path";
    std::string odometry_topic = (argc == 5) ? argv[4] : "/odometry/imu";

    std::cout << "Input bag: " << input_bag_path << std::endl;
    std::cout << "Output bag: " << output_bag_path << std::endl;
    std::cout << "Ground truth topic: " << ground_truth_topic << std::endl;
    std::cout << "Odometry topic: " << odometry_topic << std::endl;

    RosbagWithGT rosbag_with_gt(ground_truth_topic, odometry_topic);

    if (!rosbag_with_gt.readRosbag(input_bag_path)) {
        std::cerr << "Failed to read input bag." << std::endl;
        return 1;
    }

    if (!rosbag_with_gt.writeRosbag(output_bag_path)) {
        std::cerr << "Failed to write output bag: " << output_bag_path << std::endl;
        return 1;
    }

    std::cout << "Successfully processed bag file." << std::endl;

    rclcpp::shutdown();
    return 0;
}
