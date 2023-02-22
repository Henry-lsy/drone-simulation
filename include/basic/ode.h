#pragma once

#include <cmath>
#include <iostream>
#include <vector>

#include <boost/array.hpp>

#include "boost/numeric/odeint.hpp"

namespace odeint = boost::numeric::odeint;

class Ode
{
public:
    Ode(const int& state_num=18, const double& dt=0.001): _state_num(state_num), _dt(dt)
    {}

    typedef boost::array<double, 18> InternalState;

    void compute(std::vector<double> inital_state, std::vector<double> dot_state)
    {
        setInternalState(inital_state);
        setDotState(dot_state);
        auto temp_state = _x;
        odeint::integrate(boost::ref(*this), _x, 0.0, _dt, _dt);
        for (int i = 0; i < _state_num; ++i)
        {
            if (std::isnan(_x[i]))
            {
            std::cout << "dump " << i << " << pos ";
            for (int j = 0; j < 22; ++j)
            {
                std::cout << temp_state[j] << " ";
            }
            std::cout << std::endl;
            _x = temp_state;
            break;
            }
        }
    }

    void operator()(const Ode::InternalState& x,
                    Ode::InternalState& dxdt, const double /* t */)
    {
       for (int idx = 0; idx < _state_num; idx++)
       {
           dxdt[idx] = _dxdt[idx];
       }
    }
    
    std::vector<double> getInternalState()
    {
        std::vector<double> x;
        
        for (int i=0; i<_x.size(); i++)
        {
            x[i] = _x[i];
        }
        return x;
    }

    void setInternalState(const std::vector<double> & internal_state)
    {
        for (int i=0; i<internal_state.size(); i++)
        {
            _x[i] = internal_state[i];
        }
    }

    void setDotState(const std::vector<double> & dot_state)
    {
        for (int i=0; i<dot_state.size(); i++)
        {
            _dxdt[i] = dot_state[i];
        }
    }

private:
    const int _state_num; 
    const double _dt;
    InternalState _x;
    InternalState _dxdt;
};