#pragma once

#include <iostream>
#include "rigid_body.h"
#include "type.h"

class Drone: public RigidBody
{
public:
    Drone() = default;
    virtual ~Drone() = default;

    virtual void init(const std::string & config_path){};
    
    Imu getImu()
    {   _imu.orientation = getQuaterniond();
        _imu.angular_velocity = getAngularVelocity();
        _imu.linear_acceleration = getAcceleration();
        return _imu; 
    }

    Odometry getOdom()
    {
        _odom.position = getPosition();
        _odom.velocity = getVelocity();
        _odom.orientation = getQuaterniond();
        _odom.angular_velocity = getAngularVelocity();
        return _odom;
    }

    void setExternalForce(Eigen::Vector3d external_force)
    {
        _external_force = external_force;
    }

    void setExternalTorque(Eigen::Vector3d external_torque)
    {
        _external_torque = external_torque;
    }

    virtual void computeForce()
    {
        _total_force << 0, 0, 0;
    }

    virtual void computeTorque()
    {
        _total_torque << 0, 0, 0;
    }

    virtual void setInput(std::vector<double> input){}

private:
    Imu _imu;
    Odometry _odom;

protected:
    Eigen::Vector3d _external_force{0.0, 0.0, 0.0};
    Eigen::Vector3d _external_torque{0.0, 0.0, 0.0};
};