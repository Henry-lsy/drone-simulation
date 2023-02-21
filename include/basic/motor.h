#pragma once
#include <yaml-cpp/yaml.h>

class Motor
{
public:
    Motor() = default;
    Motor(const double & kf, const double & km):
        _kf(kf), _km(km)
    {}
    void setKf(double kf){ _kf = kf; }
    void setKm(double km){ _km = km; }

    double getForce();
    double getTorque();

private:
    double _rpm;
    double _kf;
    double _km;
};