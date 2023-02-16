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
    std::vector<Motor> motor;
    double arm_length;                             
};
