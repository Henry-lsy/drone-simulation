#pragma once

#include <iostream>
#include "rigid_body.h"
#include "type.h"

class Drone: public RigidBody
{
public:
    Drone() = default;
    virtual ~Drone() = default;

    virtual void init(const std::string & config_path)
    {}
    
    Imu getImu(){ return _imu; }
    Odometry getOdom(){ return _odom; }

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

    void setCmd();
    void simulationOnce(){};

private:
    Imu _imu;
    Odometry _odom;

protected:
    Eigen::Vector3d _external_force{0.0, 0.0, 0.0};
    Eigen::Vector3d _external_torque{0.0, 0.0, 0.0};
};