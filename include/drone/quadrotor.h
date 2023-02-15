#pragma once

#include "drone.h"
#include "motor.h"

class Quadrotor: public Drone
{
public:
    Quadrotor() = default;
    void init();
private:
    std::vector<Motor> motor(4);
    double arm_length;
    Eigen
};