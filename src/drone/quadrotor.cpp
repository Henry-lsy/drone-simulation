#include <yaml-cpp/yaml.h>
#include "quadrotor.h"
#include <Eigen/Core>

void Quadrotor::init(const std::string& config_path)
{
    YAML::Node config = YAML::LoadFile(config_path);

    _mass = config["mass"].as<double>();
    _inertia = Eigen::Map<Eigen::Matrix3d>(config["inertial"].as<std::vector<double>>().data());
    _arm_length = config["arm_length"].as<double>();

    // initial motor
    double kf = config["kf"].as<double>();
    double km = config["km"].as<double>();

    State initial_state;
    initial_state.position = Eigen::Map<Eigen::Vector3d>(config["initial_position"].as<std::vector<double>>().data());


    for(int i=0; i<_MOTOR_NUM; i++)
    {
      _motor[i].setKf(kf);
      _motor[i].setKm(km);
    }
}

void Quadrotor::computeTotalForce()
{
  Eigen::Vector3d motor_force{0.0, 0.0, 0.0};
  for (int i = 0; i < _MOTOR_NUM; i++)
  {
    motor_force += _motor[i].getForce() * _state.R.col(2);
  }

  Eigen::Vector3d drag_force;
  Eigen::Vector3d velocity_normalize = _state.velocity;

  if (velocity_normalize.norm() != 0)
  {
    velocity_normalize.normalize();
  }

  double resistance = 0.1 * 3.14159265 * (_arm_length) * (_arm_length) *
                      _state.velocity.norm() * _state.velocity.norm();
  drag_force = -1 * resistance * velocity_normalize;

  _total_force = motor_force + drag_force + _external_force;
}

void Quadrotor::computeTotalTorque()
{
  Eigen::Vector3d motor_torque;
  motor_torque(0) = (_motor[2].getForce() - _motor[3].getForce()) * _arm_length;
  motor_torque(1) = (_motor[1].getForce() - _motor[0].getForce()) * _arm_length;
  motor_torque(2) = _motor[0].getTorque() + _motor[1].getTorque()
                    - _motor[2].getTorque() - _motor[3].getTorque();

  _total_torque = motor_torque + _external_torque;
}

void Quadrotor::setInput(std::vector<double> rpms)
{
  for (int i=0; i<_MOTOR_NUM; i++)
  {
    _motor[i].setRpm(rpms[i]);
  }
}