#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <cfloat>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "laser_geometry/laser_geometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"


constexpr double D_R = M_PI / 180.;
constexpr double R_D = 180. / M_PI;
constexpr double g = 9.81007;

struct State {
    Eigen::Isometry3d pose;
    Eigen::Matrix<double ,6, 1> velocity;
};

using StatePtr = std::shared_ptr<State>;

struct ScanData {
    double timestamp;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;
};

using ScanDataPtr = std::shared_ptr<ScanData>;

inline pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::msg::PointCloud2 &cloudmsg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud_dst;
    pcl::fromROSMsg(cloudmsg, cloud_dst);
    return cloud_dst;
}

inline sensor_msgs::msg::PointCloud2 laser2cloudmsg(sensor_msgs::msg::LaserScan::SharedPtr laser, std::string frame_id = "scan")
{
    static laser_geometry::LaserProjection projector;
    sensor_msgs::msg::PointCloud2 pc2_dst;
    projector.projectLaser(*laser, pc2_dst,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    pc2_dst.header.frame_id = frame_id;

    return pc2_dst;
}

inline Eigen::Matrix4d inverseSE3(const Eigen::Matrix4d& T) {
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);

    // 역행렬 계산
    Eigen::Matrix4d T_inv = Eigen::Matrix4d::Identity();
    T_inv.block<3, 3>(0, 0) = R.transpose();
    T_inv.block<3, 1>(0, 3) = -R.transpose() * t;

    return T_inv;
}

#endif
