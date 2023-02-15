#include <ros/ros.h>
#include "sim_mng_ros.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "drone_simulation");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(1000);
    SimManagerRos sim_manager_ros(nh);
    sim_manager_ros.initDrone("../config/quadrotor_param.yaml");

    while(ros::ok())
    {
        sim_manager_ros.process();
        ros::spinOnce();
        loop_rate.sleep();
    }
}