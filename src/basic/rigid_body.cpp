#include <iostream>

#include "boost/numeric/odeint.hpp"

#include "rigid_body.h"
namespace odeint = boost::numeric::odeint;

void RigidBody::step(double dt)
{
  auto save = _internal_state;

  odeint::integrate(boost::ref(*this), _internal_state, 0.0, dt, dt);

  for (int i = 0; i < 18; ++i)
  {
    if (std::isnan(_internal_state[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      _internal_state = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    _state.position(i) = _internal_state[0 + i];
    _state.velocity(i) = _internal_state[3 + i];
    _state.R(i, 0) = _internal_state[6 + i];
    _state.R(i, 1) = _internal_state[9 + i];
    _state.R(i, 2) = _internal_state[12 + i];
    _state.angular_velocity(i) = _internal_state[15 + i];
  }

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(_state.R.transpose() * _state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = _state.R * P.inverse();
  _state.R                      = R;

  // Don't go below zero, simulate floor
  if (_state.position(2) < 0.0 && _state.velocity(2) < 0)
  {
    _state.position(2) = 0;
    _state.velocity(2) = 0;
  }
  updateInternalState();
}

void RigidBody::updateInternalState()
{
  for (int i = 0; i < 3; i++)
  {
    _internal_state[0 + i]  = _state.position(i);
    _internal_state[3 + i]  = _state.velocity(i);
    _internal_state[6 + i]  = _state.R(i, 0);
    _internal_state[9 + i]  = _state.R(i, 1);
    _internal_state[12 + i] = _state.R(i, 2);
    _internal_state[15 + i] = _state.angular_velocity(i);
  }
}

void RigidBody::operator()(const RigidBody::InternalState& x,
                               RigidBody::InternalState& dxdt, const double /* t */)
{
  State cur_state;
  for (int i = 0; i < 3; i++)
  {
    cur_state.position(i) = x[0 + i];
    cur_state.velocity(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.angular_velocity(i) = x[15 + i];
  }

  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d velocity, acceleration, angular_acceleration;
  Eigen::Matrix3d R_dot;
  Eigen::Vector3d vnorm;
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

  omega_vee(2, 1) = cur_state.angular_velocity(0);
  omega_vee(1, 2) = -cur_state.angular_velocity(0);
  omega_vee(0, 2) = cur_state.angular_velocity(1);
  omega_vee(2, 0) = -cur_state.angular_velocity(1);
  omega_vee(1, 0) = cur_state.angular_velocity(2);
  omega_vee(0, 1) = -cur_state.angular_velocity(2);

  velocity = cur_state.velocity;
  acceleration = -Eigen::Vector3d(0, 0, _gravity) + _total_force / _mass;

  R_dot = R * omega_vee;

  angular_acceleration = _inertia.inverse() * (_total_torque - cur_state.angular_velocity.cross(_inertia * cur_state.angular_velocity));

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = velocity(i);
    dxdt[3 + i]  = acceleration(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = angular_acceleration(i);
  }
  for (int i = 0; i < 18; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
    }
  }
}