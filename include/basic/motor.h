#pragma once

class Motor
{
public:
    double getForce();
    double getTorque();

private:
    double _rpm;
    double _thrust_coefficient;
    double _torque_coefficient;
};