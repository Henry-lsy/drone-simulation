#pragma once

#include "type.h"
#include "rigid_body.h"

class Drone: public RigidBody
{
public:
    Drone() = default;
    Imu getImu(){ return _imu; }
    Odometry getOdometry(){ return _odom; }

private:
    Imu _imu;
    Odometry _odom;
};