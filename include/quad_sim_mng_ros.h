#pragma once

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#include "quadrotor.h"

class QuadSimManagerRos
{
public:
    QuadSimManagerRos() = default;

    QuadSimManagerRos(ros::NodeHandle nh): _nh(nh)
    {
        _cmd_sub = _nh.subscribe("cmd", 100, &cmdCallback,  ros::TransportHints().tcpNoDelay());
        _imu_pub = _nh.advertise<sensor_msgs::Imu>("imu", 100);
        _odom_pub = _nh.advertise<nav_msgs::Odemetry>("odom", 100);

        _force_disturbance_sub =
            n.subscribe("force_disturbance", 100, &forceDisturbanceCallback,
                        ros::TransportHints().tcpNoDelay());
        _torque_disturbance_sub =
            n.subscribe("_torque_disturbance_sub", 100, &torqueDisturbanceCallback,
                        ros::TransportHints().tcpNoDelay());
    }
    
    void initQuadrotor(std::string config_path)
    {
        _config_path = config_path;
        _quadrotor.init(config_path);
    }
    
private:
    void cmdCallback(const geometry_msgs::Vector4::ConstPtr& cmd);

    void forceDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& force);
    void torqueDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& torque);

    ros::NodeHandler _nh;
    ros::Subscriber _cmd_sub;
    ros::Publisher _imu_pub， _odm_pub;

    ros::Subscriber _force_disturbance_sub;
    ros::Subscriber _torque_disturbance_sub;
    
    std::string _config_path;
    Quadrotor _quadrotor;
}