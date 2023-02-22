#include <iostream>

#include "rigid_body.h"

namespace odeint = boost::numeric::odeint;

void RigidBody::computeOnce()
{
  std::vector<double> initial_state = transStateToOdeState();
  std::vector<double> dot_state = computeDotState();
  _ode_int.compute(initial_state, dot_state);
  getStateFromOdeState();

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
}

void RigidBody::getStateFromOdeState()
{
  std::vector<double> x(18);
  x = _ode_int.getInternalState();

  for (int i = 0; i < 3; i++)
  {
    _state.position(i) = x[0 + i];
    _state.velocity(i) = x[3 + i];
    _state.R(i, 0) = x[6 + i];
    _state.R(i, 1) = x[9 + i];
    _state.R(i, 2) = x[12 + i];
    _state.angular_velocity(i) = x[15 + i];
  }
}

std::vector<double> RigidBody::transStateToOdeState()
{
  std::vector<double> x(18);
  for (int i = 0; i < 3; i++)
  {
    x[0 + i]  = _state.position(i);
    x[3 + i]  = _state.velocity(i);
    x[6 + i]  = _state.R(i, 0);
    x[9 + i]  = _state.R(i, 1);
    x[12 + i] = _state.R(i, 2);
    x[15 + i] = _state.angular_velocity(i);
  }
  return x;
}

std::vector<double> RigidBody::computeDotState()
{
  std::vector<double> x = _ode_int.getInternalState();
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

  std::vector<double> dxdt(18);
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
  return dxdt;
}