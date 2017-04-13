#include "heads/head_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HeadPlugin)

HeadPlugin::HeadPlugin() : server_("simple_marker") {
    node_ = new ros::NodeHandle("~");

}

void HeadPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    std::cerr << "\nThe head plugin is attach to model[" << _model->GetName() << "]\n";

    static_pose_ = _model->GetWorldPose();
    /*ROS_INFO_STREAM(static_pose_);
    static_pose_.pos.x = 2;
    _model->SetWorldPose(static_pose_);*/
    CreateMarker();

    update_ = event::Events::ConnectWorldUpdateBegin(boost::bind(\
                    &HeadPlugin::Update, this, _1));

    model_ = _model;


    if (_model->GetJointCount() == 0) {
        std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
        return;
    }



    right_eye_joint_ = model_->GetJoint("right_eye_joint");
    left_eye_joint_ = model_->GetJoint("left_eye_joint");
    ROS_INFO_STREAM("a: " << right_eye_joint_->GetAngle(0) \
          << " b: " << right_eye_joint_->GetAngle(1) \
          << " b: " << right_eye_joint_->GetAngle(2));

}

void HeadPlugin::Update(const common::UpdateInfo &_info) {
    //ROS_INFO_STREAM("<--- ");
    server_.applyChanges();
}




void HeadPlugin::CreateMarker() {
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
    server_.setCallback(head_marker_.name, boost::bind(&HeadPlugin::processFeedback, this, _1));

    server_.applyChanges();


}

void HeadPlugin::statePublisher(){
    //joint_pub_.publish(joint_msg_);
    broadcaster_.sendTransform(odom_trans_);
}


void HeadPlugin::processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) {

    odom_trans_.header.stamp = ros::Time::now();
    odom_trans_.transform.translation.x = feedback->pose.position.x;
    odom_trans_.transform.translation.y = feedback->pose.position.y;
    odom_trans_.transform.translation.z = feedback->pose.position.z;
    odom_trans_.transform.rotation = feedback->pose.orientation;

    /*ROS_INFO_STREAM(static_pose_);*/
    static_pose_.pos.x = feedback->pose.position.x;
    static_pose_.pos.y = feedback->pose.position.y;
    static_pose_.pos.z = feedback->pose.position.z;
    static_pose_.rot.x = feedback->pose.orientation.x;
    static_pose_.rot.y = feedback->pose.orientation.y;
    static_pose_.rot.z = feedback->pose.orientation.z;
    static_pose_.rot.w = feedback->pose.orientation.w;

    model_->SetWorldPose(static_pose_);

    static_pose_ = model_->GetWorldPose();
    geometry_msgs::Pose pose;
    pose.position.x = static_pose_.pos.x;
    pose.position.y = static_pose_.pos.y;
    pose.position.z = static_pose_.pos.z;
    pose.orientation.x = static_pose_.rot.x;
    pose.orientation.y = static_pose_.rot.y;
    pose.orientation.z = static_pose_.rot.z;
    pose.orientation.w = static_pose_.rot.w;

    server_.setPose(head_marker_.name, pose);
    server_.applyChanges();

    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
}
