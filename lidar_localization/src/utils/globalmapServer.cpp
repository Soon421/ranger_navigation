#include "utils/globalmapServer.h"
#include <iomanip>
#include <sstream>

GlobalmapServer::GlobalmapServer() : Node("globalmap_server")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("globalmap_pcd", " ");
    this->declare_parameter<std::vector<double>>("globalmap_xyzrpy", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    this->declare_parameter<float>("globalmap_downsample_resolution", 0.1);
    this->declare_parameter<int>("voxelmap_min_points_per_voxel", 0);

    this->get_parameter("globalmap_pcd", globalmap_pcd);
    std::vector<double> globalmap_xyzrpy_double;
    this->get_parameter("globalmap_xyzrpy", globalmap_xyzrpy_double);
    this->get_parameter("globalmap_downsample_resolution", globalmap_downsample_resolution);
    this->get_parameter("voxelmap_min_points_per_voxel", voxelmap_min_points_per_voxel);

    // Convert double vector to float vector
    globalmap_xyzrpy.resize(globalmap_xyzrpy_double.size());
    for (size_t i = 0; i < globalmap_xyzrpy_double.size(); ++i) {
        globalmap_xyzrpy[i] = static_cast<float>(globalmap_xyzrpy_double[i]);
    }

    // Create publishers
    globalmap_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", rclcpp::QoS(5).transient_local());
    voxelmap_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/voxelmap", rclcpp::QoS(5).transient_local());

    // Create timer that fires once after 1 second
    globalmap_pub_timer = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&GlobalmapServer::pub_once_cb, this));

    // Create service
    srvSaveMap = this->create_service<std_srvs::srv::Empty>(
        "~/save_map",
        std::bind(&GlobalmapServer::save_map, this, std::placeholders::_1, std::placeholders::_2));

    load_globalmap();
}

void GlobalmapServer::load_globalmap()
{
    // read globalmap from a pcd file
    globalmap.reset(new pcl::PointCloud<PointType>());
    voxelmap.reset(new pcl::PointCloud<PointType>());

    std::string extension = globalmap_pcd.substr(globalmap_pcd.find_last_of(".") + 1);
    if (extension == "ply") {
        if (pcl::io::loadPLYFile(globalmap_pcd, *globalmap) == -1) {
            std::cerr << "Failed to load PLY file: " << globalmap_pcd << std::endl;
            return;
        }
    } else if (extension == "pcd") {
        if (pcl::io::loadPCDFile(globalmap_pcd, *globalmap) == -1) {
            std::cerr << "Failed to load PCD file: " << globalmap_pcd << std::endl;
            return;
        }
    } else {
        std::cerr << "Unsupported file format: " << extension << std::endl;
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points from %s", globalmap->size(), globalmap_pcd.c_str());

    // 글로벌 맵과 occupancy 맵 위치 보정
    pcl::PointCloud<PointType>::Ptr globalmap_transed(new pcl::PointCloud<PointType>());
    int cloudSize = globalmap->size();
    globalmap_transed->resize(cloudSize);
    globalmap_transed->header.frame_id = "map";

    Eigen::Affine3f transCur = pcl::getTransformation(globalmap_xyzrpy[0], globalmap_xyzrpy[1], globalmap_xyzrpy[2],
                                                      globalmap_xyzrpy[3], globalmap_xyzrpy[4], globalmap_xyzrpy[5]);

    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = globalmap->points[i];
        globalmap_transed->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        globalmap_transed->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        globalmap_transed->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        globalmap_transed->points[i].intensity = pointFrom.intensity;
    }

    globalmap.reset(new pcl::PointCloud<PointType>());

    // downsampling large globalmap
    auto start = std::chrono::high_resolution_clock::now();

    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
    if (globalmap_downsample_resolution > 0)
    {
        boost::shared_ptr<pcl::CustomVoxelGrid<PointType>> voxelgrid(new pcl::CustomVoxelGrid<PointType>());
        voxelgrid->setLeafSize(globalmap_downsample_resolution, globalmap_downsample_resolution, globalmap_downsample_resolution);
        voxelgrid->setInputCloud(globalmap_transed);
        voxelgrid->setMinimumPointsNumberPerVoxel(voxelmap_min_points_per_voxel);
        voxelgrid->filter(*filtered);
    }
    else
    {
        filtered = globalmap_transed;
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    RCLCPP_INFO(this->get_logger(), "Global map downsampling took: %.3fs", elapsed.count());

    globalmap = filtered;

    size_t cloud_size = sizeof(PointType) * globalmap->points.size() / (1024.0 * 1024.0);
    RCLCPP_INFO(this->get_logger(), "Size of Globalmap message: %zuMB", cloud_size);


    // Iterate through the filtered points
    for (const auto &point : filtered->points)
    {
        int voxel_index_x = static_cast<int>(floor(point.x / globalmap_downsample_resolution));
        int voxel_index_y = static_cast<int>(floor(point.y / globalmap_downsample_resolution));
        int voxel_index_z = static_cast<int>(floor(point.z / globalmap_downsample_resolution));

        PointType voxel_center;
        voxel_center.x = (voxel_index_x + 0.5f) * globalmap_downsample_resolution;
        voxel_center.y = (voxel_index_y + 0.5f) * globalmap_downsample_resolution;
        voxel_center.z = (voxel_index_z + 0.5f) * globalmap_downsample_resolution;

        voxelmap->push_back(voxel_center);
    }

    filtered->header.frame_id = "map";
    voxelmap->header.frame_id = "map";
}

bool GlobalmapServer::save_map(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
    (void)req;  // Unused parameter
    (void)res;  // Unused parameter

    std::string downsampled_map_path = globalmap_pcd; // Copy the original map_path to new_map_path
    size_t pos = downsampled_map_path.find("raw");

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << globalmap_downsample_resolution;

    if (pos != std::string::npos) {
        downsampled_map_path.erase(pos);
        downsampled_map_path += "res_" + stream.str() + ".pcd";
        pcl::io::savePCDFileBinary(downsampled_map_path, *globalmap);
        RCLCPP_INFO(this->get_logger(), "Map saved successfully at: %s", downsampled_map_path.c_str());
    }

    return true;
}

void GlobalmapServer::pub_once_cb()
{
    // Convert PCL to ROS2 message
    sensor_msgs::msg::PointCloud2 globalmap_msg;
    sensor_msgs::msg::PointCloud2 voxelmap_msg;

    pcl::toROSMsg(*globalmap, globalmap_msg);
    pcl::toROSMsg(*voxelmap, voxelmap_msg);

    globalmap_msg.header.frame_id = "map";
    globalmap_msg.header.stamp = this->now();
    voxelmap_msg.header.frame_id = "map";
    voxelmap_msg.header.stamp = this->now();

    globalmap_pub->publish(globalmap_msg);
    voxelmap_pub->publish(voxelmap_msg);

    // Cancel the timer after first execution
    globalmap_pub_timer->cancel();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GlobalmapServer>();

    RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Global Map Server Started.\033[0m");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
