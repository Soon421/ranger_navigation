#include "utility.hpp"
#include "lidar_localization/msg/cloud_info.hpp"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include "mapMatching.h"
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;


class localizationOptimization : public ParamServer
{

public:

    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    MapMatching *mapMatching;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gps_integration_position_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_matching_integration_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_matching_initial_pose_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr map_matching_rejected_pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_lidar_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr initial_pose_received_pub;

    rclcpp::Subscription<lidar_localization::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    lidar_localization::msg::CloudInfo cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D_map_matching;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D_map_matching;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D_initial_pose;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D_initial_pose;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::CustomVoxelGrid<PointType> downSizeFilterCorner;
    pcl::CustomVoxelGrid<PointType> downSizeFilterSurf;
    pcl::CustomVoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;
    bool global_pose_initialized = false;
    bool initial_pose_received = false;

    float transformTobeMapped[6];

    std::mutex mtx;
    std::mutex globalmap_mtx;
    std::mutex map_matching_mtx;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    // TF2 for baselink to lidar transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    geometry_msgs::msg::TransformStamped baselink2lidar_transform_;
    bool has_baselink2lidar_ = false;

    // Map matching
    pcl::PointCloud<PointType>::Ptr globalmap;
    pcl::PointCloud<pcl::PointXY>::Ptr globalmap_xy;
    lidar_localization::msg::CloudInfo cloud_info_last;
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr globalmap_xy_kdtree;
    int map_matching_index_last = -1;
    gtsam::Pose3 map_matching_pose_last;
    gtsam::Matrix6 map_matching_covariance_last;

    // Initial pose
    int initial_pose_index_last = -1;
    gtsam::Pose3 initial_pose_last;
    gtsam::noiseModel::Diagonal::shared_ptr initial_pose_covariance_last;
    geometry_msgs::msg::Pose initial_pose_msg_last;

    // Parameters
    double map_matching_update_interval;
    vector<double> matching_covariance_const;
    double overlap_ratio_threshold;
    double map_matching_stale_time_threshold;
    std::string ndt_neighbor_search_method;
    int number_of_threads_ndt;
    double ndt_resolution;
    double lidar_downsample_resolution;

    localizationOptimization(const rclcpp::NodeOptions & options) : ParamServer("lidar_localization_localizationOptimization", options)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>("lidar_localization/mapping/trajectory", 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>("lidar_localization/mapping/odometry", qos);
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
            "lidar_localization/mapping/odometry_incremental", qos);
        pubPath = create_publisher<nav_msgs::msg::Path>("lidar_localization/mapping/path", 1);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Initialize TF2 buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Lookup baselink to lidar transform if frames are different
        if(lidarFrame != baselinkFrame)
        {
            try
            {
                // Wait for transform to become available
                rclcpp::sleep_for(std::chrono::seconds(1));
                baselink2lidar_transform_ = tf_buffer_->lookupTransform(
                    lidarFrame, baselinkFrame,
                    tf2::TimePointZero,
                    tf2::durationFromSec(3.0)
                );
                has_baselink2lidar_ = true;
                RCLCPP_INFO(this->get_logger(), "Successfully obtained baselink to lidar transform");
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to get baselink to lidar transform: %s", ex.what());
                has_baselink2lidar_ = false;
            }
        }

        subCloud = create_subscription<lidar_localization::msg::CloudInfo>(
            "lidar_localization/feature/cloud_info", qos,
            std::bind(&localizationOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subGPS = create_subscription<nav_msgs::msg::Odometry>(
            gpsTopic, 200,
            std::bind(&localizationOptimization::gpsHandler, this, std::placeholders::_1));

        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>("lidar_localization/mapping/map_local", 1);
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>("lidar_localization/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>("lidar_localization/mapping/cloud_registered_raw", 1);

        // Additional publishers for localization
        gps_integration_position_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/gps_integration_position", 5);
        map_matching_initial_pose_pub = create_publisher<nav_msgs::msg::Odometry>("/map_matching_inititial_pose", 5);
        map_matching_integration_pose_pub = create_publisher<nav_msgs::msg::Odometry>("/map_matching_integration_pose", 5);
        map_matching_rejected_pose_pub = create_publisher<nav_msgs::msg::Odometry>("/map_matching_rejected_pose", 5);
        aligned_lidar_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
        initial_pose_received_pub = create_publisher<std_msgs::msg::Bool>("lidar_localization/initial_pose_received", 5);

        // Additional subscribers for localization
        // Disable intraprocess for transient_local compatibility
        rclcpp::SubscriptionOptions globalmap_sub_options;
        globalmap_sub_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

        globalmap_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/globalmap", rclcpp::QoS(5).transient_local(),
            std::bind(&localizationOptimization::globalmapHandler, this, std::placeholders::_1),
            globalmap_sub_options);

        initialpose_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&localizationOptimization::initialposeHandler, this, std::placeholders::_1));

        // Load parameters for map matching
        declare_parameter("map_matching_update_interval", 2.0);
        declare_parameter("matching_covariance_const", std::vector<double>{0.1, 0.1, 0.01, 0.001, 0.001, 0.001});
        declare_parameter("overlap_ratio_threshold", 0.6);
        declare_parameter("map_matching_stale_time_threshold", 20.0);
        declare_parameter("ndt_neighbor_search_method", "DIRECT7");
        declare_parameter("number_of_threads_ndt", 8);
        declare_parameter("ndt_resolution", 1.0);
        declare_parameter("lidar_downsample_resolution", 0.1);

        get_parameter("map_matching_update_interval", map_matching_update_interval);
        get_parameter("matching_covariance_const", matching_covariance_const);
        get_parameter("overlap_ratio_threshold", overlap_ratio_threshold);
        get_parameter("map_matching_stale_time_threshold", map_matching_stale_time_threshold);
        get_parameter("ndt_neighbor_search_method", ndt_neighbor_search_method);
        get_parameter("number_of_threads_ndt", number_of_threads_ndt);
        get_parameter("ndt_resolution", ndt_resolution);
        get_parameter("lidar_downsample_resolution", lidar_downsample_resolution);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization

        allocateMemory();
        initializeMapMatching();

        // Initialize globalmap (only once, not during re-localization)
        globalmap.reset(new pcl::PointCloud<PointType>());
        globalmap_xy.reset(new pcl::PointCloud<pcl::PointXY>());
        globalmap_xy_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXY>());
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D_map_matching.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D_map_matching.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D_initial_pose.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D_initial_pose.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP.setZero();

        // Reset GTSAM graph and estimates
        aLoopIsClosed = false;
        gtSAMgraph.resize(0);
        initialEstimate.clear();
        optimizedEstimate.clear();
        isamCurrentEstimate.clear();
        poseCovariance.resize(6, 6);

        // Clear queues and containers
        gpsQueue.clear();
        cornerCloudKeyFrames.clear();
        surfCloudKeyFrames.clear();
        laserCloudMapContainer.clear();

        // Clear path
        globalPath.poses.clear();

        // Reset map matching related variables
        initial_pose_index_last = -1;
        map_matching_index_last = -1;
        map_matching_pose_last = gtsam::Pose3();

        // Initialize map matching covariance
        map_matching_covariance_last = gtsam::Matrix6::Identity();
        map_matching_covariance_last(0, 0) = matching_covariance_const[3];
        map_matching_covariance_last(1, 1) = matching_covariance_const[4];
        map_matching_covariance_last(2, 2) = matching_covariance_const[5];
        map_matching_covariance_last(3, 3) = matching_covariance_const[0];
        map_matching_covariance_last(4, 4) = matching_covariance_const[1];
        map_matching_covariance_last(5, 5) = matching_covariance_const[2];

        initial_pose_last = gtsam::Pose3();
        initial_pose_covariance_last = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6(matching_covariance_const.data()));
    }

    void initializeMapMatching() {
        std::lock_guard<std::mutex> lock(map_matching_mtx);
        mapMatching = new MapMatching();
        mapMatching->setNDTResolution(ndt_resolution);
        mapMatching->setTransformationEpsilon(0.01);
        mapMatching->setMaxIteration(35);
        mapMatching->setNumThreads(number_of_threads_ndt);
        mapMatching->setNDTNeighborSearchMethod(ndt_neighbor_search_method);
        mapMatching->setLidarDownsampleResolution(lidar_downsample_resolution);
    }

    void globalmapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr points_msg) {
        RCLCPP_INFO(this->get_logger(), "globalmap received!");
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*points_msg, *cloud);

        auto t1 = std::chrono::high_resolution_clock::now();
        {
            std::lock_guard<std::mutex> lock(map_matching_mtx);
            mapMatching->setInputTarget(cloud);

            // Perform dummy alignment to initialize NDT internal structures
            pcl::PointCloud<PointType>::Ptr temp_source(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr temp_aligned(new pcl::PointCloud<PointType>());
            PointType temp_point;
            temp_point.x = 0.0;
            temp_point.y = 0.0;
            temp_point.z = 0.0;
            temp_source->push_back(temp_point);
            mapMatching->setInputSource(temp_source);
            mapMatching->align(temp_aligned, Eigen::Matrix4f::Identity());
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        RCLCPP_INFO_STREAM(this->get_logger(), "globalmap Initialization finished! Taken time: " << duration / 1000.0 << "s");

        globalmap = cloud;

        pcl::PointCloud<pcl::PointXY>::Ptr globalmap_xy_temp(new pcl::PointCloud<pcl::PointXY>());
        for (const auto& point : cloud->points)
        {
            pcl::PointXY point_xy;
            point_xy.x = point.x;
            point_xy.y = point.y;
            globalmap_xy_temp->push_back(point_xy);
        }
        globalmap_mtx.lock();
        globalmap_xy = globalmap_xy_temp;
        globalmap_xy_kdtree->setInputCloud(globalmap_xy);
        globalmap_mtx.unlock();
    }

    void initialposeHandler(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg) {
        global_pose_initialized = false;

        if (globalmap->empty())
        {
            RCLCPP_WARN(this->get_logger(), "globalmap has not been received!!");
            return;
        }

        globalmap_mtx.lock();
        if (globalmap_xy->empty())
        {
            RCLCPP_WARN(this->get_logger(), "globalmap_xy is empty!!");
            globalmap_mtx.unlock();
            return;
        }

        if (!globalmap_xy_kdtree || globalmap_xy_kdtree->getInputCloud() == nullptr) {
            RCLCPP_WARN(this->get_logger(), "globalmap_xy_kdtree has not been initialized!!");
            globalmap_mtx.unlock();
            return;
        }
        globalmap_mtx.unlock();

        // Find nearest point in globalmap to get Z coordinate
        std::vector<int> globalmap_xy_nearest_index;
        std::vector<float> globalmap_xy_nearest_dist;
        pcl::PointXY initial_pose_xy;
        initial_pose_xy.x = pose_msg->pose.pose.position.x;
        initial_pose_xy.y = pose_msg->pose.pose.position.y;

        globalmap_xy_kdtree->nearestKSearch(initial_pose_xy, 1, globalmap_xy_nearest_index, globalmap_xy_nearest_dist);
        PointType nearest_point = globalmap->points[globalmap_xy_nearest_index[0]];

        // Set initial pose with Z coordinate from globalmap
        geometry_msgs::msg::Pose initial_pose_geometry = pose_msg->pose.pose;
        initial_pose_geometry.position.z = nearest_point.z;

        // Convert base_link pose in map to os_lidar pose in map:
        //   T_{map<-lidar} = T_{map<-base} * T_{base<-lidar}
        // baselink2lidar_transform_ is T_{lidar<-base}, so right-multiply by its inverse.
        if(has_baselink2lidar_)
        {
            tf2::Transform t_lidar_base;
            tf2::fromMsg(baselink2lidar_transform_.transform, t_lidar_base);
            tf2::Transform t_base_lidar = t_lidar_base.inverse();

            tf2::Transform t_map_base;
            tf2::fromMsg(initial_pose_geometry, t_map_base);

            tf2::Transform t_map_lidar = t_map_base * t_base_lidar;
            tf2::toMsg(t_map_lidar, initial_pose_geometry);
            RCLCPP_INFO(this->get_logger(), "Applied baselink to lidar transform to initial pose");
        }

        mtx.lock();
        initial_pose_received = true;
        initial_pose_msg_last = initial_pose_geometry;

        // Re-initialize ISAM2 for re-localization
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        allocateMemory();
        mtx.unlock();

        std_msgs::msg::Bool initial_pose_received_msg;
        initial_pose_received_msg.data = true;
        initial_pose_received_pub->publish(initial_pose_received_msg);

        RCLCPP_INFO(this->get_logger(), "Initial pose received from RViz");
    }

    void mapMatchingThread()
    {
        rclcpp::Rate rate(1.0/map_matching_update_interval);
        while (rclcpp::ok())
        {
            rate.sleep();
            performMapmatching();
        }
    }

    void initialPoseThread()
    {
        rclcpp::Rate rate(10);
        while (rclcpp::ok())
        {
            rate.sleep();
            if (!global_pose_initialized && initial_pose_msg_last.position.x != 0)
            {
                initialFactorGeneration(initial_pose_msg_last);
            }
        }
    }


    void initialFactorGeneration(geometry_msgs::msg::Pose initial_pose_geometry)
    {
        mtx.lock();
        *copy_cloudKeyPoses3D_initial_pose = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D_initial_pose = *cloudKeyPoses6D;
        mtx.unlock();

        if (copy_cloudKeyPoses3D_initial_pose->points.empty() == true)
            return;
            
        int initialPoseKeyCur = copy_cloudKeyPoses3D_initial_pose->size() - 1;

        Eigen::Matrix4f initial_pose_matrix = geometryPose2Matrix4f(initial_pose_geometry);

        lidar_localization::msg::CloudInfo copy_cloud_info_last;
        mtx.lock();
        copy_cloud_info_last = cloud_info_last;
        mtx.unlock();

        Eigen::Matrix4f initial_matching_pose_matrix;
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(map_matching_mtx);
            mapMatching->setInputSource(copy_cloud_info_last.cloud_deskewed);
            mapMatching->alignWithParticle(aligned, initial_pose_matrix, 21, 11, 4.0, 10.0);
            initial_matching_pose_matrix = mapMatching->getFinalTransformation();

            sensor_msgs::msg::PointCloud2 aligned_msg;
            pcl::toROSMsg(*aligned, aligned_msg);
            aligned_msg.header.frame_id = "map";
            aligned_msg.header.stamp = copy_cloud_info_last.header.stamp;
            aligned_lidar_pub->publish(aligned_msg);

            mapMatching->calculateProbability(aligned);
        }
        double overlap_ratio = mapMatching->getOverlapRatio();
        if(overlap_ratio < overlap_ratio_threshold)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "initial pose overlap ratio : " << overlap_ratio << "  -> rejection!");
            mtx.lock();
            initial_pose_index_last = -1;
            mtx.unlock();
            return;
        }

        noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, 1e-2, 5.0, 5.0, 5.0).finished());
        mtx.lock();
        initial_pose_index_last = initialPoseKeyCur;
        initial_pose_last = eigenMatrix2gtsamPose(initial_matching_pose_matrix);
        initial_pose_covariance_last = priorNoise;

        // Reset initial_pose_msg_last to prevent duplicate calls
        initial_pose_msg_last.position.x = 0;
        initial_pose_msg_last.position.y = 0;
        initial_pose_msg_last.position.z = 0;
        mtx.unlock();
    }

    void performMapmatching()
    {
        if(globalmap->empty()) {
            RCLCPP_WARN(this->get_logger(), "globalmap has not been received!!");
            return;
        }

        if (!global_pose_initialized)
            return;

        lidar_localization::msg::CloudInfo copy_cloud_info_last;
        mtx.lock();
        *copy_cloudKeyPoses3D_map_matching = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D_map_matching = *cloudKeyPoses6D;
        copy_cloud_info_last = cloud_info_last;
        mtx.unlock();

        if (copy_cloudKeyPoses3D_map_matching->points.empty() == true)
            return;

        static double last_map_matching_time = -1;
        if (initial_pose_received)
            last_map_matching_time = -1;

        int mapMatchingKeyCur = copy_cloudKeyPoses3D_map_matching->size() - 1;

        if (mapMatchingKeyCur == map_matching_index_last)
            return;

        if (copy_cloud_info_last.cloud_deskewed.width < 3000)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "map matching cloud size is too small : " << copy_cloud_info_last.cloud_deskewed.width);
            return;
        }

        Eigen::Matrix4f matching_init_pose = pclPointToAffine3f(copy_cloudKeyPoses6D_map_matching->points[mapMatchingKeyCur]).matrix();
        pcl::PointCloud<PointType>::Ptr aligned(new pcl::PointCloud<PointType>());
        Eigen::Matrix4f matching_pose_matrix;
        {
            std::lock_guard<std::mutex> lock(map_matching_mtx);
            mapMatching->setInputSource(copy_cloud_info_last.cloud_deskewed);
            mapMatching->align(aligned, matching_init_pose);
            matching_pose_matrix = mapMatching->getFinalTransformation();
        }

        nav_msgs::msg::Odometry matching_pose_msg, matching_init_pose_msg;
        geometry_msgs::msg::Pose matching_pose_geom, matching_init_pose_geom;
        matching_pose_geom = tf2::toMsg(Eigen::Isometry3d(matching_pose_matrix.cast<double>()));
        matching_pose_msg.pose.pose = matching_pose_geom;
        matching_init_pose_geom = tf2::toMsg(Eigen::Isometry3d(matching_init_pose.cast<double>()));
        matching_init_pose_msg.pose.pose = matching_init_pose_geom;

        Eigen::Matrix<double, 6, 6> matching_covariance_local = Eigen::Matrix<double, 6, 6>::Identity();
        for (size_t i = 0; i < 6; i++)
        {
            matching_covariance_local(i,i) = matching_covariance_const[i];
        }

        Eigen::Matrix3d rotation = matching_pose_matrix.cast<double>().block<3,3>(0,0);
        Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Zero();
        J.block<3,3>(0,0) = rotation;
        J.block<3,3>(3,3) = rotation;
        Eigen::Matrix<double,6,6> matching_covariance_global = J * matching_covariance_local * J.transpose();

        for (size_t i = 0; i < 6; i++)
        {
            for (size_t j = 0; j < 6; j++)
            {
                matching_pose_msg.pose.covariance[6 * i + j] = matching_covariance_global(i, j);
            }
        }

        sensor_msgs::msg::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_msg.header.frame_id = "map";
        aligned_msg.header.stamp = copy_cloud_info_last.header.stamp;
        aligned_lidar_pub->publish(aligned_msg);

        double overlap_ratio;
        {
            std::lock_guard<std::mutex> lock(map_matching_mtx);
            mapMatching->calculateProbability(aligned);
            overlap_ratio = mapMatching->getOverlapRatio();
        }

        if (map_matching_initial_pose_pub->get_subscription_count() > 0)
        {
            matching_init_pose_msg.header.frame_id = "map";
            matching_init_pose_msg.header.stamp = copy_cloud_info_last.header.stamp;
            map_matching_initial_pose_pub->publish(matching_init_pose_msg);
        }
        static nav_msgs::msg::Odometry matching_pose_msg_last;

        double current_time = stamp2Sec(copy_cloud_info_last.header.stamp);
        if(overlap_ratio < overlap_ratio_threshold
            && current_time - last_map_matching_time < map_matching_stale_time_threshold)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "map matching overlap ratio : " << overlap_ratio);
            if(map_matching_rejected_pose_pub->get_subscription_count() > 0)
            {
                matching_pose_msg.header.frame_id = "map";
                matching_pose_msg.header.stamp = copy_cloud_info_last.header.stamp;
                map_matching_rejected_pose_pub->publish(matching_pose_msg);
            }
            matching_covariance_local(0,0) = 1e10;
            matching_covariance_local(1,1) = 1e10;
            matching_covariance_local(2,2) = 0.1;
            matching_covariance_local(3,3) = 0.01;
            matching_covariance_local(4,4) = 0.01;
            matching_covariance_local(5,5) = 4*M_PI*M_PI;
        }
        else
        {
            if(map_matching_integration_pose_pub->get_subscription_count() > 0)
            {
                matching_pose_msg.header.frame_id = "map";
                matching_pose_msg.header.stamp = copy_cloud_info_last.header.stamp;
                map_matching_integration_pose_pub->publish(matching_pose_msg);
            }
            last_map_matching_time = current_time;
        }

        Eigen::Matrix<double, 6, 6> matching_covariance_rpyxyz;
        matching_covariance_rpyxyz.block<3,3>(0,0) = matching_covariance_local.block<3,3>(3,3);
        matching_covariance_rpyxyz.block<3,3>(3,0) = matching_covariance_local.block<3,3>(0,3);
        matching_covariance_rpyxyz.block<3,3>(0,3) = matching_covariance_local.block<3,3>(3,0);
        matching_covariance_rpyxyz.block<3,3>(3,3) = matching_covariance_local.block<3,3>(0,0);

        Matrix6 noise_covariance_matrix = matching_covariance_rpyxyz;
        mtx.lock();
        map_matching_index_last = mapMatchingKeyCur;
        map_matching_pose_last = eigenMatrix2gtsamPose(matching_pose_matrix);
        map_matching_covariance_last = noise_covariance_matrix;
        mtx.unlock();

        matching_pose_msg_last = matching_pose_msg;
    }

    void laserCloudInfoHandler(const lidar_localization::msg::CloudInfo::SharedPtr msgIn)
    {
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = stamp2Sec(msgIn->header.stamp);

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (initial_pose_received)
            timeLastProcessing = -1;

        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }

        initial_pose_received = false;
    }

    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    gtsam::Pose3 eigenMatrix2gtsamPose(const Eigen::Matrix4f& eigen_matrix) {
        Eigen::Matrix3f rot = eigen_matrix.block<3, 3>(0, 0);
        Eigen::Vector3f trans = eigen_matrix.block<3, 1>(0, 3);

        return gtsam::Pose3(gtsam::Rot3(rot.cast<double>()), gtsam::Point3(trans.cast<double>()));
    }

    Eigen::Matrix4f geometryPose2Matrix4f(geometry_msgs::msg::Pose geometry_pose)
    {
        float x, y, z, roll, pitch, yaw;
        double roll_d, pitch_d, yaw_d;

        tf2::Quaternion orientation;
        tf2::fromMsg(geometry_pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll_d, pitch_d, yaw_d);

        x = geometry_pose.position.x;
        y = geometry_pose.position.y;
        z = geometry_pose.position.z;
        roll = roll_d;
        pitch = pitch_d;
        yaw = yaw_d;

        Eigen::Affine3f transform = pcl::getTransformation(x, y, z, roll, pitch, yaw);
        return transform.matrix();
    }




    



    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;

        static int buffer = 0;
        buffer++;

        // Reset IMU pre-integration when initial pose is received (re-localization)
        if (initial_pose_received)
        {
            lastImuPreTransAvailable = false;
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            buffer = 0;
        }

        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imu_roll_init;
            transformTobeMapped[1] = cloudInfo.imu_pitch_init;
            transformTobeMapped[2] = cloudInfo.imu_yaw_init;

            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        if (cloudInfo.odom_available == true)
        {
            Eigen::Affine3f transBack;
            if (buffer > 2)
                transBack = pcl::getTransformation(cloudInfo.initial_guess_x,    cloudInfo.initial_guess_y,     cloudInfo.initial_guess_z,
                                                   cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            else if (!useImuHeadingInitialization)
                transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, 0);
            else
                transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);

            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imu_available == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
            return;
        }
    }


    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for(auto& pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
            {
                // transformed cloud available
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
            } else {
                // transformed cloud not available
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap   += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            int found = kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (found < 5) {
                // 이웃이 5개 미만이면 스킵
                continue;
            }

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));
                    
            if (pointSearchSqDis[4] < 1.0) {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5; cy /= 5;  cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
                    a22 += ay * ay; a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5; a12 /= 5; a13 /= 5; a22 /= 5; a23 /= 5; a33 /= 5;

                matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                                    + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                                    + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                              + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                               - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                               + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel); 
            int found = kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
            if (found < 5) {
                // 이웃이 5개 미만이면 스킵
                continue;
            }

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }

                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps; pb /= ps; pc /= ps; pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                            + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum && 
            laserCloudCornerFromMapDSNum > 0 && laserCloudSurfFromMapDSNum > 0)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
            incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imu_available == true)
        {
            if (std::abs(cloudInfo.imu_pitch_init) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        if (sensor == SensorType::LIVOX)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
                return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw)   < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x*x + y*y + z*z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty() && !initial_pose_received)
        {
            // First keyframe without initial pose - use weak prior
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 4*M_PI*M_PI, 4*M_PI*M_PI, 4*M_PI*M_PI, 1e10, 1e10, 1e10).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else if (cloudKeyPoses3D->points.empty() && initial_pose_received)
        {
            // First keyframe with initial pose from RViz - use initial pose as prior
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 4*M_PI*M_PI, 4*M_PI*M_PI, 4*M_PI*M_PI, 1e10, 1e10, 1e10).finished()); // rad*rad, meter*meter
            gtsam::Pose3 initial_pose;
            initial_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(initial_pose_msg_last.orientation.w, initial_pose_msg_last.orientation.x, initial_pose_msg_last.orientation.y, initial_pose_msg_last.orientation.z),
                                        gtsam::Point3(initial_pose_msg_last.position.x, initial_pose_msg_last.position.y, initial_pose_msg_last.position.z));
            gtSAMgraph.add(PriorFactor<Pose3>(0, initial_pose, priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            // Subsequent keyframes - use odometry between factor
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-4, 1e-4, 1e-4, 1e-3, 1e-3, 1e-3).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        static double last_gps_integration_time = -1;
        static int gps_integrated_count = 0;

        // Reset GPS integration state when initial pose is received (re-localization)
        if (initial_pose_received)
        {
            last_gps_integration_time = -1;
            gps_integrated_count = 0;
        }

        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        double no_gps_integration_period = timeLaserInfoCur - last_gps_integration_time;

        // pose covariance small, no need to correct
        if ((poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold) && no_gps_integration_period < gps_update_interval)
            return;

        // last gps position
        static PointType lastGPSPoint;
        if (initial_pose_received)
        {
            lastGPSPoint.x = 0;
            lastGPSPoint.y = 0;
            lastGPSPoint.z = 0;
        }

        while (!gpsQueue.empty())
        {
            if (stamp2Sec(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (stamp2Sec(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                    continue;
                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!useGpsElevation && global_pose_initialized)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 100.0;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                Eigen::Vector4d cur_gps_xyz_time;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                cur_gps_xyz_time << gps_x, gps_y, gps_z, stamp2Sec(timeLaserInfoStamp);
                if (pointDistance(curGPSPoint, lastGPSPoint) < 3.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                if (!global_pose_initialized)
                {
                    gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                    gtSAMgraph.add(gps_factor);
                }
                else
                {
                    // Check GPS distance for outlier rejection
                    double gps_dist = sqrt(pow(transformTobeMapped[3] - gps_x, 2) + pow(transformTobeMapped[4] - gps_y, 2));
                    if(gps_dist > gps_distance_threshold)
                    {
                        RCLCPP_WARN_STREAM(this->get_logger(), "gps_dist : " << gps_dist << "  -> rejection!");
                        break;
                    }

                    gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                    gtSAMgraph.add(gps_factor);
                }

                pcl::PointCloud<PointType>::Ptr gps_pointcloud(new pcl::PointCloud<PointType>());
                gps_pointcloud->push_back(curGPSPoint);

                publishCloud(pubCloudRegisteredRaw, gps_pointcloud, timeLaserInfoStamp, mapFrame);

                last_gps_integration_time = stamp2Sec(thisGPS.header.stamp);

                aLoopIsClosed = true;
                RCLCPP_INFO_STREAM(this->get_logger(), "GPS update!");
                gps_integrated_count++;
                if(!global_pose_initialized && gps_integrated_count == 2)
                {
                    global_pose_initialized = true;
                    RCLCPP_INFO_STREAM(this->get_logger(), "Global pose initialized!");
                }

                break;
            }
        }
    }

    void addMapmatchingFactor()
    {
        if (map_matching_index_last < 0)
            return;

        if (cloudKeyPoses3D->points.size() < map_matching_index_last)
        {
            map_matching_index_last = -1;
            return;
        }

        int index = map_matching_index_last;
        gtsam::Pose3 mapMatchingPose = map_matching_pose_last;
        gtsam::Matrix6 mapMatchingCovariance = map_matching_covariance_last;
        gtSAMgraph.add(PriorFactor<Pose3>(index, mapMatchingPose, noiseModel::Gaussian::Covariance(mapMatchingCovariance)));

        // RCLCPP_INFO_STREAM(this->get_logger(), "Map matching pose updated!");

        map_matching_index_last = -1;
        aLoopIsClosed = true;
    }

    void addInitialposeFactor()
    {
        if (initial_pose_index_last < 0 || global_pose_initialized)
            return;

        if (cloudKeyPoses3D->points.size() < initial_pose_index_last)
        {
            initial_pose_index_last = -1;
            return;
        }

        gtsam::Pose3 initialPose = initial_pose_last;
        gtsam::noiseModel::Diagonal::shared_ptr initialPoseCovariance = initial_pose_covariance_last;
        gtSAMgraph.add(PriorFactor<Pose3>(initial_pose_index_last, initialPose, initialPoseCovariance));

        RCLCPP_INFO_STREAM(this->get_logger(), "Global pose initialized!");

        initial_pose_index_last = -1;
        aLoopIsClosed = true;
        global_pose_initialized = true;
    }

    
    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false && initial_pose_index_last < 0)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // add initial pose prior factor
        addInitialposeFactor();

        // map matching factor
        addMapmatchingFactor();


        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        cloud_info_last = cloudInfo;

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::msg::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        tf2::Quaternion quat_tf;
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        geometry_msgs::msg::Quaternion quat_msg;
        tf2::convert(quat_tf, quat_msg);
        laserOdometryROS.pose.pose.orientation = quat_msg;
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish TF
        quat_tf.setRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        tf2::Transform t_odom_to_lidar = tf2::Transform(quat_tf, tf2::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf2::TimePoint time_point = tf2_ros::fromRclcpp(timeLaserInfoStamp);
        tf2::Stamped<tf2::Transform> temp_odom_to_lidar(t_odom_to_lidar, time_point, odometryFrame);
        geometry_msgs::msg::TransformStamped trans_odom_to_lidar;
        tf2::convert(temp_odom_to_lidar, trans_odom_to_lidar);
        trans_odom_to_lidar.child_frame_id = "lidar_link";
        br->sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (initial_pose_received)
            lastIncreOdomPubFlag = false;

        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
            if (initial_pose_received)
                increOdomAffine = pcl::getTransformation(0, 0, 0, 0, 0, 0);

        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (global_pose_initialized)
            {
                double currentPoseWeight = 0.3;
                tf2::Quaternion currentPoseQuaternion;
                tf2::Quaternion increOdomQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                currentPoseQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                increOdomQuaternion.setRPY(roll, 0, 0);
                tf2::Matrix3x3(currentPoseQuaternion.slerp(increOdomQuaternion, currentPoseWeight)).getRPY(rollMid, pitchMid, yawMid);
                roll = rollMid;

                // slerp pitch
                currentPoseQuaternion.setRPY(0, transformTobeMapped[1], 0);
                increOdomQuaternion.setRPY(0, pitch, 0);
                tf2::Matrix3x3(currentPoseQuaternion.slerp(increOdomQuaternion, currentPoseWeight)).getRPY(rollMid, pitchMid, yawMid);
                pitch = pitchMid;

                increOdomAffine = pcl::getTransformation(x, y, z, roll, pitch, yaw);
            }
            else if (cloudInfo.imu_available == true)
            {
                if (std::abs(cloudInfo.imu_pitch_init) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;

                    increOdomAffine = pcl::getTransformation(x, y, z, roll, pitch, yaw);
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath->publish(globalPath);
        }
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto MO = std::make_shared<localizationOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread map_matching_thread(&localizationOptimization::mapMatchingThread, MO);
    std::thread initial_pose_thread(&localizationOptimization::initialPoseThread, MO);

    exec.spin();

    rclcpp::shutdown();

    map_matching_thread.join();
    initial_pose_thread.join();

    return 0;
}
