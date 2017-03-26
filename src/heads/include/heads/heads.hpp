#ifndef HEADS_HPP
#define HEADS_HPP

#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>


class HeadController {

public:

    HeadController(ros::NodeHandle &nodehandle);

    void update();
    void statePublisher();
    void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

private:

    ros::NodeHandle node_;
    ros::Publisher joint_pub_;
    sensor_msgs::JointState joint_msg_;
    tf::TransformBroadcaster broadcaster_;
    geometry_msgs::TransformStamped odom_trans_;


    interactive_markers::InteractiveMarkerServer server_;
    visualization_msgs::InteractiveMarker head_marker_;


};


#endif
