#pragma once

#include <Eigen/Geometry>

struct State
{
    Eigen::Vector3d position(0, 0, 0);
    Eigen::Vector3d velocity(0, 0, 0);
    Eigen::Vector3d acceleration(0, 0, 0);
    Eigen::Vector3d jerk(0, 0, 0);
    Eigen::Vector3d snap(0, 0, 0);

    Eigen::Quaterniond orientation(0, 0, 0, 1);
    Eigen::Vector3d angular_velocity(0, 0, 0);
    Eigen::Vector3d angular_acceleration(0, 0, 0);
}

struct Imu
{
    // maybe add time stamp
    Eigen::Quaterniond orientation(0, 0, 0, 1);
    Eigen::Vector3d angular_velocity(0, 0, 0);
    Eigen::Vector3d linear_acceleration(0, 0, 0);
}

struct Odometry
{
    Eigen::Vector3d position(0, 0, 0);
    Eigen::Quaterniond orientation(0, 0, 0, 1);
    
    Eigen::Vector3d velocity(0, 0, 0);
    Eigen::Vector3d angular_velocity(0, 0, 0);
}