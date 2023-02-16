#pragma once

#include <string>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include "quadrotor_msgs/PositionCommand.h"

#include "quadrotor.h"

#include <ros/ros.h>

class SimManagerRos
{
public:
    SimManagerRos() = default;

    SimManagerRos(ros::NodeHandle& nh): _nh(nh)
    {
        _cmd_sub = _nh.subscribe<quadrotor_msgs::PositionCommand>("/cmd", 100, &SimManagerRos::cmdCallback, this, ros::TransportHints().tcpNoDelay());
        _imu_pub = _nh.advertise<sensor_msgs::Imu>("imu", 100);
        _odom_pub = _nh.advertise<nav_msgs::Odometry>("odom", 100);

        _force_disturbance_sub =
            _nh.subscribe<geometry_msgs::Vector3>("force_disturbance", 100, &SimManagerRos::forceDisturbanceCallback, this, ros::TransportHints().tcpNoDelay());
        _torque_disturbance_sub =
            _nh.subscribe<geometry_msgs::Vector3>("_torque_disturbance_sub", 100, &SimManagerRos::torqueDisturbanceCallback, this, ros::TransportHints().tcpNoDelay());
    }
    
    void initDrone(std::string config_path)
    {
        _config_path = config_path;
        
        _drone.init(config_path);
    }

    void process()
    {
        _drone.simulationOnce();
        publishImu();
        publishOdom();
    }

    void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
    {
        // _drone.setCmd();
    }
private:
    void forceDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& force)
    {
        // _drone.setExtraForce(force);
    }

    void torqueDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& torque)
    {
        // _drone.setExtraTorque(torque);
    }

    void publishOdom()
    {
        nav_msgs::Odometry odom_data;
        // odom_data = _drone.getOdom();
        _odom_pub.publish(odom_data);
    }

    void publishImu()
    {
        sensor_msgs::Imu imu_data;
        // imu_data = _drone.getImu();
        _imu_pub.publish(imu_data);
    }

    ros::NodeHandle _nh;
    ros::Subscriber _cmd_sub;
    ros::Publisher _imu_pub, _odom_pub;

    ros::Subscriber _force_disturbance_sub;
    ros::Subscriber _torque_disturbance_sub;
    
    std::string _config_path;
    Drone _drone;  // can be generate to class Drone later
};