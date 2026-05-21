#include "mapMatching.h"

// Static member initialization
const std::map<std::string, pclomp::NeighborSearchMethod> MapMatching::methodMap_ = {
    {"DIRECT1", pclomp::DIRECT1},
    {"DIRECT7", pclomp::DIRECT7},
    {"DIRECT26", pclomp::DIRECT26},
    {"KDTREE", pclomp::KDTREE}
};


MapMatching::MapMatching() :
    ndt_(new pclomp::NormalDistributionsTransform<PointType, PointType>()),
    ndt_neighbor_search_method_("DIRECT7"), ndt_resolution_(1.0), transformation_elipson_(0.01), max_iteration_(35),
    number_of_threads_ndt_score_(8), lidar_downsample_resolution_(0.2)
{
    initializeNDT();
    lidar_downsample_filter_.setLeafSize(lidar_downsample_resolution_, lidar_downsample_resolution_, lidar_downsample_resolution_);

    input_cloud_ = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
}

void MapMatching::initializeNDT()
{
    ndt_->setTransformationEpsilon(transformation_elipson_);
    ndt_->setResolution(ndt_resolution_);
    ndt_->setMaximumIterations(max_iteration_);
    ndt_->setNumThreads(number_of_threads_ndt_score_);

    auto it = methodMap_.find(ndt_neighbor_search_method_);
    if (it != methodMap_.end()) {
        ndt_->setNeighborhoodSearchMethod(it->second);
        std::cout << "search_method " << ndt_neighbor_search_method_ << " is selected" << std::endl;
    } else {
        std::cout << "invalid search method was given" << std::endl;
        std::cout << "default method is selected (KDTREE)" << std::endl;
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
}

void MapMatching::setInputTarget(pcl::PointCloud<PointType>::Ptr map) {
    globalmap_ = map;
    ndt_->setInputTarget(globalmap_);
}

void MapMatching::setInputSource(const sensor_msgs::msg::PointCloud2& cloud) {
    pcl::PointCloud<PointType>::Ptr scan(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());

    pcl::fromROSMsg(cloud, *scan);
    lidar_downsample_filter_.setInputCloud(scan);
    lidar_downsample_filter_.filter(*filtered);

    input_cloud_ = filtered;
    ndt_->setInputSource(filtered);
}

void MapMatching::setInputSource(const pcl::PointCloud<PointType>::Ptr& scan) {
    pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());

    lidar_downsample_filter_.setInputCloud(scan);
    lidar_downsample_filter_.filter(*filtered);

    input_cloud_ = filtered;
    ndt_->setInputSource(filtered);
}

void MapMatching::align(pcl::PointCloud<PointType>::Ptr& output_cloud, const Eigen::Matrix4f& initial_pose) {
    output_cloud->clear();
    ndt_->align(*output_cloud, initial_pose);
}

void MapMatching::alignWithParticle(pcl::PointCloud<PointType>::Ptr& output_cloud, const Eigen::Matrix4f& initial_pose,
                                    const int& particle_num_trans, const int& particle_num_angle,
                                    const double& particle_max_trans, const double& particle_max_angle) {
    output_cloud->clear();
    pcl::PointCloud<PointType>::Ptr particle_cloud(new pcl::PointCloud<PointType>());
    double interval_trans = 0.0, interval_angle = 0.0;
    if(particle_num_trans > 1)
        interval_trans = 2 * particle_max_trans / (particle_num_trans - 1);

    if(particle_num_angle > 1)
        interval_angle = 2 * particle_max_angle / (particle_num_angle - 1);

    Eigen::Matrix4f max_probability_pose = initial_pose;
    double max_probability = 0.0;

    for (int i= 0 ; i < particle_num_trans; i++)
    {
        for (int j= 0 ; j < particle_num_trans; j++)
        {
            for (int k= 0 ; k < particle_num_angle; k++)
            {
                double x_inc = 0.0, y_inc = 0.0, yaw_inc = 0.0;
                Eigen::Affine3f particle_pose_affine = Eigen::Affine3f(initial_pose);
                Eigen::Matrix4f particle_pose;

                if(particle_num_trans > 1)
                {
                    x_inc = - particle_max_trans + interval_trans * i;
                    y_inc = - particle_max_trans + interval_trans * j;
                }
                if(particle_num_angle > 1)
                {
                    yaw_inc = (- particle_max_angle + interval_angle * k) * M_PI / 180.0;
                }

                Eigen::Affine3f particle_inc = pcl::getTransformation(x_inc, y_inc, 0, 0, 0, yaw_inc);
                particle_pose_affine =  particle_pose_affine * particle_inc;
                particle_pose = particle_pose_affine.matrix();

                pcl::transformPointCloud(*input_cloud_, *particle_cloud, particle_pose);
                double probability = ndt_->calculateProbability(*particle_cloud);
                if(probability > max_probability)
                {
                    max_probability = probability;
                    max_probability_pose = particle_pose;
                }
            }
        }
    }

    ndt_->align(*output_cloud, max_probability_pose);
}

Eigen::Matrix4f MapMatching::getFinalTransformation() const {
    return ndt_->getFinalTransformation();
}



bool MapMatching::hasConverged() const {
    return ndt_->hasConverged();
}

double MapMatching::getFitnessScore() const {
    return ndt_->getFitnessScore();
}

double MapMatching::calculateProbability(const pcl::PointCloud<PointType>::Ptr& cloud) const {
    return ndt_->calculateProbability(*cloud);
}

double MapMatching::getOverlapRatio() const {
    return ndt_->getOverlapRatio();
}

// Setter Methods
void MapMatching::setNDTNeighborSearchMethod(const std::string& method) {
    auto it = methodMap_.find(method);
    if (it != methodMap_.end()) {
        ndt_neighbor_search_method_ = method;
        ndt_->setNeighborhoodSearchMethod(it->second);
        std::cout << "search_method " << method << " is selected" << std::endl;
    } else {
        std::cout << "invalid search method was given" << std::endl;
        std::cout << "default method is selected (KDTREE)" << std::endl;
        ndt_neighbor_search_method_ = "KDTREE"; // Fallback to default
        ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
    }
}

void MapMatching::setTransformationEpsilon(double epsilon) {
    transformation_elipson_ = epsilon;
    ndt_->setTransformationEpsilon(epsilon);
}

void MapMatching::setMaxIteration(int maxIter) {
    max_iteration_ = maxIter;
    ndt_->setMaximumIterations(maxIter);
}

void MapMatching::setNDTResolution(double resolution) {
    ndt_resolution_ = resolution;
    ndt_->setResolution(resolution);
}

void MapMatching::setNumThreads(int numThreads) {
    number_of_threads_ndt_score_ = numThreads;
    ndt_->setNumThreads(numThreads);
}

void MapMatching::setLidarDownsampleResolution(double resolution) {
    lidar_downsample_resolution_ = resolution;
    lidar_downsample_filter_.setLeafSize(lidar_downsample_resolution_, lidar_downsample_resolution_, lidar_downsample_resolution_);
}
