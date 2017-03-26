#include <ros/ros.h>
#include "heads/heads.hpp"



int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_marker");
    ros::NodeHandle node;

    HeadController head(node);
    ros::Rate loop_rate(30);

    // start the ROS main loop
    while(ros::ok()){
        head.update();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
