#pragma once
#include <Eigen/Dense>
#include "type.h"

typedef boost::array<double, 22> InternalState;

class RigidBody
{
public:
    RigidBody() = default;
    RigidBody(double mass, Eigen::Matrix3d inertia):_mass(mass), _inertia(inertia)
    {}
    virtual ~RigidBody() = default;

    void compute();
    void setState(const State & state){ _state = state; }

    Eigen::Vector3d getPosition(){ return _state.position; }
    Eigen::Vector3d getVelocity(){ return _state.velocity; }
    Eigen::Vector3d getAcceleration(){ return _state.acceleration; }

    Eigen::Matrix3d getRotationMatix();
    Eigen::Quaterniond getQuaterniond(){ return _state.orientation; }

    Eigen::Vector3d getAngularVelocity(){ return _state.angular_velocity; }
    Eigen::Vector3d getAngularAcceleration(){ return _state.angular_acceleration; }

    // used for change structure or add other payload directly.
    void setMass(double mass){ _mass = mass; }
    void setInertia(const Eigen::Matrix3d & inertia) { _inertia = inertia; }

protected:
    double _mass;
    Eigen::Matrix3d _inertia;
    Eigen::Vector3d _total_force;
    Eigen::Vector3d _total_torque;

    State _state;

private:
    InternalState _internal_state;
};