#include <yaml-cpp/yaml.h>
#include "quadrotor.h"
#include <Eigen/Core>

void Quadrotor::init(const std::string& config_path): 
{
    YAML::Node config = YAML::LoadFile(config_path);

    _mass = config["mass"].as<double>();
    _inertia = Eigen::Map<Eigen::Matrix3d>(config["inertial"].as<std::vector<double>>().data());

    double kf = config["kf"].as<double>();
    double km = config["km"].as<double>();

    for(int i=0; i<_motor_num; i++)
    {
      _motor[i].setKf(kf);
      _motor[i].setKm(km);
    }

}

void Quadrotor::computeTotalForce()
{
  // Eigen::Vector3d force
}

void Quadrotor::computeTotalTorque()
{
  Eigen::Vector3d torque;
  torque(0) = (_motor[2].getForce() - _motor[3].getForce()) * _arm_length;
  torque(1) = (_motor[1].getForce() - _motor[0].getForce()) * _arm_length;
  torque(2) = _motor[0].getTorque() + _motor[1].getTorque()
              - _motor[2].getTorque() - _motor[3].getTorque();

  // double resistance = 0.1 *                                        // C
  //                     3.14159265 * (_arm_length) * (_arm_length) * // S
  //                     cur_state.v.norm() * cur_state.v.norm();
}