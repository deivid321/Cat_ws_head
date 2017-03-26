#include "heads/heads.hpp"

HeadController::HeadController(ros::NodeHandle &nodehandle): server_("simple_marker"), node_(nodehandle){

    //joint_pub_ = node_.advertise<sensor_msgs::JointState>("joint_states", 1);

    odom_trans_.header.frame_id = "world";
    odom_trans_.child_frame_id = "base_link";
    odom_trans_.header.stamp = ros::Time::now();
    odom_trans_.transform.translation.x = 0;
    odom_trans_.transform.translation.y = 0;
    odom_trans_.transform.translation.z = 0;
    odom_trans_.transform.rotation = tf::createQuaternionMsgFromYaw(0);


    head_marker_.header.frame_id = "base_link";
    head_marker_.header.stamp = ros::Time::now();
    head_marker_.name = "head_marker";
    head_marker_.description = "6-DOF Control of head";


    // create a grey sphere marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.type = visualization_msgs::Marker::SPHERE;
    sphere_marker.scale.x = 0.45;
    sphere_marker.scale.y = 0.45;
    sphere_marker.scale.z = 0.45;
    sphere_marker.color.r = 0.5;
    sphere_marker.color.g = 0.5;
    sphere_marker.color.b = 0.5;
    sphere_marker.color.a = 1.0;

    // create a non-interactive control which contains the sphere
    visualization_msgs::InteractiveMarkerControl sphere_control;
    sphere_control.always_visible = true;
    sphere_control.markers.push_back(sphere_marker);

    // add the control to the interactive marker
    head_marker_.controls.push_back(sphere_control);

    //visualization_msgs::InteractiveMarkerControl rotate_control;
    //rotate_control.name = "move_x";
    //rotate_control.interaction_mode =
    //    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    head_marker_.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    head_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    head_marker_.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    head_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    head_marker_.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    head_marker_.controls.push_back(control);



    // add the control to the interactive marker
    head_marker_.controls.push_back(control);

    // tell the server to call processFeedback() when feedback arrives for it
    //server_.insert(head_marker_, &processFeedback);
    server_.insert(head_marker_);
    server_.setCallback(head_marker_.name, boost::bind(&HeadController::processFeedback, this, _1));

    server_.applyChanges();

}


void HeadController::update(){
    statePublisher();
    server_.applyChanges();
}

void HeadController::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {

    odom_trans_.header.stamp = ros::Time::now();
    odom_trans_.transform.translation.x = feedback->pose.position.x;
    odom_trans_.transform.translation.y = feedback->pose.position.y;
    odom_trans_.transform.translation.z = feedback->pose.position.z;
    odom_trans_.transform.rotation = feedback->pose.orientation;


    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
}


void HeadController::statePublisher(){
    //joint_pub_.publish(joint_msg_);
    broadcaster_.sendTransform(odom_trans_);
}
