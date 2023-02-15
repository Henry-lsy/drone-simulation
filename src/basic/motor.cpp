#include "motor.h"

Motor::getForce(const double& rpm)
{
    return _thrust_coefficient * rpm * rpm;
}

Motor::getTorque(const double& rpm)
{
    return _torque_coefficient * rpm * rpm;
}