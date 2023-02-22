#pragma once
#include <yaml-cpp/yaml.h>

#include "ode.h"

class Motor
{
public:
    Motor() = default;
    Motor(const double & kf, const double & km):
        _kf(kf), _km(km)
    {}
    void setKf(double kf){ _kf = kf; }
    void setKm(double km){ _km = km; }
    void setRpm(double rpm){ _rpm = rpm;}

    double getForce();
    double getTorque();

private:
    double _rpm;
    double _kf;
    double _km;

    std::vector<double> computeDotState();
    std::vector<double> transStateToOdeState();
    void getStateFromOdeState();
    Ode ode_motor;
};