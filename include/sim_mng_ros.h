#pragma once

#include <ros/ros.h>
#include <string>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "quadrotor_msgs/ControlCommand.h"

#include "quadrotor.h"
#include "type.h"

class SimManagerRos
{
public:
    SimManagerRos() = default;

    SimManagerRos(ros::NodeHandle& nh): _nh(nh)
    {
        _cmd_sub = _nh.subscribe<quadrotor_msgs::ControlCommand>("/cmd", 100, &SimManagerRos::cmdCallback, this, ros::TransportHints().tcpNoDelay());
        _imu_pub = _nh.advertise<sensor_msgs::Imu>("imu", 100);
        _odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 100);

        _force_disturbance_sub =
            _nh.subscribe<geometry_msgs::Vector3>("force_disturbance", 100, &SimManagerRos::forceDisturbanceCallback, this, ros::TransportHints().tcpNoDelay());
        _torque_disturbance_sub =
            _nh.subscribe<geometry_msgs::Vector3>("torque_disturbance", 100, &SimManagerRos::torqueDisturbanceCallback, this, ros::TransportHints().tcpNoDelay());
    }
    
    void initDrone(std::string config_path)
    { 
        _config_path = config_path;
        // drone should be initialied properly!
        _drone_ptr->init(config_path);
    }

    void process()
    {
        _drone_ptr->computeOnce();
        publishImu();
        publishOdom();
    }

    void cmdCallback(const quadrotor_msgs::ControlCommand::ConstPtr& cmd)
    {
        _drone_ptr->setInput(cmd->rotor_rpms);
    }

private:
    void forceDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& force_msg)
    {
        Eigen::Vector3d force;
        force(0) = force_msg->x;
        force(1) = force_msg->y;
        force(2) = force_msg->z;
        _drone_ptr->setExternalForce(force);
    }

    void torqueDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& torque_msg)
    {
        Eigen::Vector3d torque;
        torque(0) = torque_msg->x;
        torque(1) = torque_msg->y;
        torque(2) = torque_msg->z;
        _drone_ptr->setExternalTorque(torque);
    }

    void publishOdom()
    {
        Odometry odom = _drone_ptr->getOdom();
        nav_msgs::Odometry odom_ros;
        odom_ros.pose.pose.position.x = odom.position[0];
        odom_ros.pose.pose.position.y = odom.position[1];
        odom_ros.pose.pose.position.z = odom.position[2];

        odom_ros.pose.pose.orientation.x = odom.orientation.x();
        odom_ros.pose.pose.orientation.y = odom.orientation.y();
        odom_ros.pose.pose.orientation.z = odom.orientation.z();
        odom_ros.pose.pose.orientation.w = odom.orientation.w();

        odom_ros.twist.twist.linear.x = odom.velocity[0];
        odom_ros.twist.twist.linear.y = odom.velocity[1];
        odom_ros.twist.twist.linear.z = odom.velocity[2];

        odom_ros.twist.twist.angular.x = odom.angular_velocity[0];
        odom_ros.twist.twist.angular.y = odom.angular_velocity[1];
        odom_ros.twist.twist.angular.z = odom.angular_velocity[2];
        _odom_pub.publish(odom_ros);
    }

    void publishImu()
    {
        Imu imu = _drone_ptr->getImu();
        sensor_msgs::Imu imu_ros;
        imu_ros.orientation.x = imu.orientation.x();
        imu_ros.orientation.y = imu.orientation.y();
        imu_ros.orientation.z = imu.orientation.z();
        imu_ros.orientation.w = imu.orientation.w();

        imu_ros.angular_velocity.x = imu.angular_velocity[0];
        imu_ros.angular_velocity.y = imu.angular_velocity[1];
        imu_ros.angular_velocity.z = imu.angular_velocity[2];

        imu_ros.linear_acceleration.x = imu.linear_acceleration[0];
        imu_ros.linear_acceleration.y = imu.linear_acceleration[1];
        imu_ros.linear_acceleration.z = imu.linear_acceleration[2];
        _imu_pub.publish(imu_ros);
    }

    ros::NodeHandle _nh;
    ros::Subscriber _cmd_sub;
    ros::Publisher _imu_pub, _odom_pub;

    ros::Subscriber _force_disturbance_sub;
    ros::Subscriber _torque_disturbance_sub;
    
    std::string _config_path;
    Drone* _drone_ptr;
};