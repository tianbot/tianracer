#include "ros/ros.h"
#include "tianboard.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tianracer");
    ros::NodeHandle nh("tianracer");
    Tianboard tianboard(&nh);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
