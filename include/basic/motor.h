#pragma once

class Motor
{
public:
    double getForce(const double& rpm);
    double getTorque(const double& rpm);

private:
    double _rpm;
    double _thrust_coefficient;
    double _torque_coefficient;
};