
#include "lidar_odometry/lidar_odometry.hpp"

LidarOdometry::LidarOdometry(double max_correspondence_distance, double transformation_epsilon, double maximum_iterations)
{
    gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setMaximumIterations(maximum_iterations);

    POSE_G_L = Eigen::Matrix4d::Zero();

    POSE_G_L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    POSE_G_L.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
    POSE_G_L(3, 3) = 1.0;

    state_ptr = std::make_shared<State>();
    state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
    state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
    state_ptr->velocity = Eigen::Matrix<double, 6, 1>::Zero();
}

StatePtr LidarOdometry::get_state()
{
    return state_ptr;
}

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp;
        Eigen::Matrix4d transform_matrix = get_transform_matrix(last_scan_ptr, scan_ptr);
        POSE_G_L = POSE_G_L * inverseSE3(transform_matrix);

        Eigen::Matrix3d rotation_matrix = POSE_G_L.block<3, 3>(0, 0);
        Eigen::AngleAxisd angleAxis(rotation_matrix);
        Eigen::Vector3d axis = angleAxis.axis();
        double angle = angleAxis.angle();

        Eigen::Vector3d translation_velocity = transform_matrix.block<3, 1>(0, 3) / dt;
        Eigen::Vector3d angular_velocity = (angle / dt) * axis;

        state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
        state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
        state_ptr->velocity.block<3, 1>(0, 0) = translation_velocity;
        state_ptr->velocity.block<3, 1>(3, 0) = angular_velocity;
    }

    last_scan_ptr = scan_ptr;
}

Eigen::Matrix4d LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

    gicp->setInputSource(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(source->point_cloud));
    gicp->setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(target->point_cloud));
    gicp->align(*align);

    Eigen::Matrix4f src2tgt = gicp->getFinalTransformation();

    return src2tgt.cast<double>();    
}
