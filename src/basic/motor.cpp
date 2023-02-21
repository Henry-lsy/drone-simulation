#include "motor.h"

double Motor::getForce()
{
    return _thrust_coefficient * _rpm * _rpm;
}

double Motor::getTorque()
{
    return _torque_coefficient * _rpm * _rpm;
}