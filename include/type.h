#pragma once

// #include <Eigen/Geometry>
#include <Eigen/Dense>
// #include <Eigen/Core>

struct State
{
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d jerk;
    Eigen::Vector3d snap;

    Eigen::Matrix3d R;
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;
};

struct Imu
{
    // maybe add time stamp
    Eigen::Quaterniond orientation;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d linear_acceleration;
};

struct Odometry
{
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    
    Eigen::Vector3d velocity;
    Eigen::Vector3d angular_velocity;
};