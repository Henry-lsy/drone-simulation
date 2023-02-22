#pragma once

#include <vector>
#include <string>

#include "drone.h"
#include "motor.h"

class Quadrotor: public Drone
{
public:
    // Quadrotor() = default;
    void init(const std::string & config_path) override;

    void computeTotalForce() override;
    void computeTotalTorque() override;

    void setInput(std::vector<double> rpms) override;
private:
    const int _MOTOR_NUM = 4;
    std::vector<Motor> _motor;
    double _arm_length;                             
};
