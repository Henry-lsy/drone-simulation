#pragma once
#include <Eigen/Geometry>
#include "type"

class RigidBody
{
public:
    RigidBody(double mass, Eigen::matrix3d inertia):_mass(mass), _inertia(inertia)
    {}

    void compute();
    void setState(const State & state){ _state = state; }

    Eigen::Vector3d getPosition(){ return _state.position; }
    Eigen::Vector3d getVelocity(){ return _state.velocity; }
    Eigen::Vector3d getAcceleration(){ return _state.acceleration; }

    Eigen::Matrix3d getRotationMatix();
    Eigen::Quaterniond getQuaterniond(){ return _state.orientation; }

    Eigen::Vector3d getAngularVelocity(){ return _state.angular_velocity; }
    Eigen::Vector3d getAngularAcceleration(){ return _state.angular_acceleration; }

private:
    double _mass;
    Eigen::Matrix3d _inertia;
    State _state;
};