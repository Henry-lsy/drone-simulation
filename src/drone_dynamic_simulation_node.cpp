#include <ros/ros.h>
#include "quadrotor.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simulation_model");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(1000);
    Quadrotor quadrotor;
    quadrotor.init("../config/quadrotor_param.yaml");
    while(ros::ok())
    {
        quadrotor.simulation_once();
        ros::spinOnce();
        loop_rate.sleep();
    }
}