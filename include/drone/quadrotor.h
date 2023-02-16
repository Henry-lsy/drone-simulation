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

    void computeForce() override;
    void computeTorque() override;
    
private:
    const int _rotor_num = 4;
    std::vector<Motor> _motors;
    double _arm_length;                             
};
