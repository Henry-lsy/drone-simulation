#include "rigid_body.h"

void RigidBody::step(double dt)
{
  auto save = internal_state_;

  odeint::integrate(boost::ref(*this), internal_state_, 0.0, dt, dt);

  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(internal_state_[i]))
    {
      std::cout << "dump " << i << " << pos ";
      for (int j = 0; j < 22; ++j)
      {
        std::cout << save[j] << " ";
      }
      std::cout << std::endl;
      internal_state_ = save;
      break;
    }
  }

  for (int i = 0; i < 3; i++)
  {
    state_.x(i) = internal_state_[0 + i];
    state_.v(i) = internal_state_[3 + i];
    state_.R(i, 0) = internal_state_[6 + i];
    state_.R(i, 1) = internal_state_[9 + i];
    state_.R(i, 2) = internal_state_[12 + i];
    state_.omega(i) = internal_state_[15 + i];
  }
  state_.motor_rpm(0) = internal_state_[18];
  state_.motor_rpm(1) = internal_state_[19];
  state_.motor_rpm(2) = internal_state_[20];
  state_.motor_rpm(3) = internal_state_[21];

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(state_.R.transpose() * state_.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = state_.R * P.inverse();
  state_.R                      = R;

  // Don't go below zero, simulate floor
  if (state_.x(2) < 0.0 && state_.v(2) < 0)
  {
    state_.x(2) = 0;
    state_.v(2) = 0;
  }
  updateInternalState();
}

void RigidBody::updateInternalState(void)
{
  for (int i = 0; i < 3; i++)
  {
    internal_state_[0 + i]  = state_.x(i);
    internal_state_[3 + i]  = state_.v(i);
    internal_state_[6 + i]  = state_.R(i, 0);
    internal_state_[9 + i]  = state_.R(i, 1);
    internal_state_[12 + i] = state_.R(i, 2);
    internal_state_[15 + i] = state_.omega(i);
  }
  internal_state_[18] = state_.motor_rpm(0);
  internal_state_[19] = state_.motor_rpm(1);
  internal_state_[20] = state_.motor_rpm(2);
  internal_state_[21] = state_.motor_rpm(3);
}
//refer to other code
void RigidBody::compute()
{
  State cur_state;
  for (int i = 0; i < 3; i++)
  {
    cur_state.x(i) = x[0 + i];
    cur_state.v(i) = x[3 + i];
    cur_state.R(i, 0) = x[6 + i];
    cur_state.R(i, 1) = x[9 + i];
    cur_state.R(i, 2) = x[12 + i];
    cur_state.omega(i) = x[15 + i];
  }
  for (int i = 0; i < 4; i++)
  {
    cur_state.motor_rpm(i) = x[18 + i];
  }

  // std::cout << "Omega: " << cur_state.omega << std::endl;
  // std::cout << "motor_rpm: " << cur_state.motor_rpm << std::endl;

  // Re-orthonormalize R (polar decomposition)
  Eigen::LLT<Eigen::Matrix3d> llt(cur_state.R.transpose() * cur_state.R);
  Eigen::Matrix3d             P = llt.matrixL();
  Eigen::Matrix3d             R = cur_state.R * P.inverse();

  Eigen::Vector3d x_dot, v_dot, omega_dot;
  Eigen::Matrix3d R_dot;
  Eigen::Array4d  motor_rpm_dot;
  Eigen::Vector3d vnorm;
  Eigen::Array4d  motor_rpm_sq;
  Eigen::Matrix3d omega_vee(Eigen::Matrix3d::Zero());

  omega_vee(2, 1) = cur_state.omega(0);
  omega_vee(1, 2) = -cur_state.omega(0);
  omega_vee(0, 2) = cur_state.omega(1);
  omega_vee(2, 0) = -cur_state.omega(1);
  omega_vee(1, 0) = cur_state.omega(2);
  omega_vee(0, 1) = -cur_state.omega(2);

  motor_rpm_sq = cur_state.motor_rpm.array().square();

  double thrust = kf_ * motor_rpm_sq.sum();

//   Eigen::Vector3d moments;
//   moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
//   moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
//   moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
//                       motor_rpm_sq(3));

//   double resistance = 0.1 *                                        // C
//                       3.14159265 * (arm_length_) * (arm_length_) * // S
//                       cur_state.v.norm() * cur_state.v.norm();

  //  ROS_INFO("resistance: %lf, Thrust: %lf%% ", resistance,
  //           motor_rpm_sq.sum() / (4 * max_rpm_ * max_rpm_) * 100.0);

  vnorm = cur_state.v;
  if (vnorm.norm() != 0)
  {
    vnorm.normalize();
  }
  x_dot = cur_state.v;
  v_dot = -Eigen::Vector3d(0, 0, g_) + thrust * R.col(2) / mass_ +
          external_force_ / mass_ /*; //*/ - resistance * vnorm / mass_;

  acc_ = v_dot;
  //  acc_[2] = -acc_[2]; // to NED

  R_dot = R * omega_vee;
  omega_dot = J_.inverse() * (moments - cur_state.omega.cross(J_ * cur_state.omega) + external_moment_);
  motor_rpm_dot = (input_ - cur_state.motor_rpm) / motor_time_constant_;

  for (int i = 0; i < 3; i++)
  {
    dxdt[0 + i]  = x_dot(i);
    dxdt[3 + i]  = v_dot(i);
    dxdt[6 + i]  = R_dot(i, 0);
    dxdt[9 + i]  = R_dot(i, 1);
    dxdt[12 + i] = R_dot(i, 2);
    dxdt[15 + i] = omega_dot(i);
  }
  for (int i = 0; i < 4; i++)
  {
    dxdt[18 + i] = motor_rpm_dot(i);
  }
  for (int i = 0; i < 22; ++i)
  {
    if (std::isnan(dxdt[i]))
    {
      dxdt[i] = 0;
    }
  }    
}