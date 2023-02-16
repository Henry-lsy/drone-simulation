#include <yaml-cpp/yaml.h>
#include "quadrotor.h"
#include <Eigen/Core>

void Quadrotor::init(const std::string& config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    _mass = config["mass"].as<double>();
    _inertia = Eigen::Map<Eigen::Matrix3d>(config["inertial"].as<std::vector<double>>().data());

}

void Quadrotor::computeForce()
{

}

void Quadrotor::computeTorque()
{
  Eigen::Vector3d moments;
  moments(0) = kf_ * (motor_rpm_sq(2) - motor_rpm_sq(3)) * arm_length_;
  moments(1) = kf_ * (motor_rpm_sq(1) - motor_rpm_sq(0)) * arm_length_;
  moments(2) = km_ * (motor_rpm_sq(0) + motor_rpm_sq(1) - motor_rpm_sq(2) -
                      motor_rpm_sq(3));

  double resistance = 0.1 *                                        // C
                      3.14159265 * (arm_length_) * (arm_length_) * // S
                      cur_state.v.norm() * cur_state.v.norm();
}