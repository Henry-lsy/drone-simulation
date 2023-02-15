#pragma once

#include <ros/ros.h>
#include <string>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>

#include "quadrotor.h"

class SimManagerRos
{
public:
    SimManagerRos() = default;

    SimManagerRos(ros::NodeHandle nh): _nh(nh)
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
    
    void initDrone(std::string config_path)
    {
        _config_path = config_path;
        _drone.init(config_path);
    }

    void process()
    {
        _drone.simulation_once();
        publish_imu();
        publish_odom();
    }
    
private:
    void cmdCallback(const geometry_msgs::Vector4::ConstPtr& cmd)
    {
        _drone.set_cmd(cmd);
    }

    void forceDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& force)
    {
        _drone.set_extra_force(force);
    }

    void torqueDisturbanceCallback(const geometry_msgs::Vector3::ConstPtr& torque);
    {
        _drone.set_extra_torque(torque);
    }

    void publish_odom()
    {
        nav_msgs::Odemetry odom_data;
        odom_data = _drone.get_odom();
        _odom_pub.publish(odom_data);
    }

    void publish_imu()
    {
        sensor_msgs::Imu imu_data;
        imu_data = _drone.get_imu();
        _imu_pub.publish(imu_data);
    }

    ros::NodeHandler _nh;
    ros::Subscriber _cmd_sub;
    ros::Publisher _imu_pub， _odom_pub;

    ros::Subscriber _force_disturbance_sub;
    ros::Subscriber _torque_disturbance_sub;
    
    std::string _config_path;
    Quadrotor _drone;  // can be generate to class Drone later
};