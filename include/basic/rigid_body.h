#pragma once
#include <Eigen/Dense>
#include <boost/array.hpp>
#include "type.h"

class RigidBody
{
public:
    typedef boost::array<double, 18> InternalState;

    RigidBody() = default;
    RigidBody(double mass, Eigen::Matrix3d inertia):_mass(mass), _inertia(inertia)
    {}
    virtual ~RigidBody() = default;

    virtual void computeTotalForce() = 0;
    virtual void computeTotalTorque() = 0;
    void operator()(const RigidBody::InternalState& x,
                    RigidBody::InternalState& dxdt, const double /* t */);
    
    void updateInternalState();
    void step(double dt);
    void setState(const State & state){ _state = state; }

    Eigen::Vector3d getPosition(){ return _state.position; }
    Eigen::Vector3d getVelocity(){ return _state.velocity; }
    Eigen::Vector3d getAcceleration(){ return _state.acceleration; }

    Eigen::Matrix3d getRotationMatix() {return _state.R; }
    Eigen::Quaterniond getQuaterniond(){ return _state.orientation; }

    Eigen::Vector3d getAngularVelocity(){ return _state.angular_velocity; }
    Eigen::Vector3d getAngularAcceleration(){ return _state.angular_acceleration; }

    // used for change structure or add other payload directly.
    void setMass(double mass){ _mass = mass; }
    void setInertia(const Eigen::Matrix3d & inertia) { _inertia = inertia; }

protected:
    double _mass;
    const double _gravity = 9.81; // This parameter should belong to class Envronment.
    Eigen::Matrix3d _inertia;
    Eigen::Vector3d _total_force;
    Eigen::Vector3d _total_torque;

    State _state;

private:
    InternalState _internal_state;
};