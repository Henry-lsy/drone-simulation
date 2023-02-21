#include "motor.h"

double Motor::getForce()
{
    return _kf * _rpm * _rpm;
}

double Motor::getTorque()
{
    return _km * _rpm * _rpm;
}