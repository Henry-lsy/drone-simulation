#include "motor.h"

double Motor::getForce(const double& rpm)
{
    return _thrust_coefficient * rpm * rpm;
}

double Motor::getTorque(const double& rpm)
{
    return _torque_coefficient * rpm * rpm;
}