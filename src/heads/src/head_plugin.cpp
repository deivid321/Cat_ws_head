#include "heads/head_plugin.h"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HeadPlugin)

HeadPlugin::HeadPlugin() : server_("simple_marker") {
    node_ = new ros::NodeHandle("~");

}

void HeadPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    std::cerr << "\nThe head plugin is attach to model[" << _model->GetName() << "]\n";

    static_pose_ = _model->GetWorldPose();

    links_ = _model->GetLinks();

    ROS_INFO_STREAM("a: " << links_.size() << ", " << links_[0]->GetName() << ", " << links_[1]->GetName()<< ", " << links_[2]->GetName());
    CreateMarker();

    update_ = event::Events::ConnectWorldUpdateBegin(boost::bind(\
                    &HeadPlugin::Update, this, _1));

    model_ = _model;

}

void HeadPlugin::Update(const common::UpdateInfo &_info) {
    //ROS_INFO_STREAM("<--- ");
    gazebo::math::Pose p0 = links_[0]->GetWorldPose();
    gazebo::math::Pose p1 = links_[1]->GetWorldPose();

    geometry_msgs::Pose pose;
    pose.position.x = p0.pos.x;
    pose.position.y = p0.pos.y;
    pose.position.z = p0.pos.z;
    pose.orientation.x = p1.rot.x;
    pose.orientation.y = p1.rot.y;
    pose.orientation.z = p1.rot.z;
    pose.orientation.w = p1.rot.w;
    setEyePose(pose);
    
    server_.applyChanges();

}

void HeadPlugin::setMarker() {
    static_pose_ = model_->GetWorldPose();
    geometry_msgs::Pose pose;
    pose.position.x = static_pose_.pos.x;
    pose.position.y = static_pose_.pos.y;
    pose.position.z = static_pose_.pos.z;
    pose.orientation.x = static_pose_.rot.x;
    pose.orientation.y = static_pose_.rot.y;
    pose.orientation.z = static_pose_.rot.z;
    pose.orientation.w = static_pose_.rot.w;
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
    gazebo::math::Pose p0 = links_[0]->GetWorldPose();
    static_pose_.pos.x = p0.pos.x;//feedback->pose.position.x;
    static_pose_.pos.y = p0.pos.y;//feedback->pose.position.y;
    static_pose_.pos.z = p0.pos.z;//feedback->pose.position.z;
    static_pose_.rot.x = feedback->pose.orientation.x;
    static_pose_.rot.y = feedback->pose.orientation.y;
    static_pose_.rot.z = feedback->pose.orientation.z;
    static_pose_.rot.w = feedback->pose.orientation.w;

    
    setEyePose(feedback->pose);
    //model_->SetWorldPose(static_pose_);
    server_.applyChanges();

    /*static_pose_ = model_->GetWorldPose();
    geometry_msgs::Pose pose;
    pose.position.x = static_pose_.pos.x;
    pose.position.y = static_pose_.pos.y;
    pose.position.z = static_pose_.pos.z;
    pose.orientation.x = static_pose_.rot.x;
    pose.orientation.y = static_pose_.rot.y;
    pose.orientation.z = static_pose_.rot.z;
    pose.orientation.w = static_pose_.rot.w;


    server_.setPose(head_marker_.name, pose);
    server_.applyChanges();*/

    ROS_INFO_STREAM( feedback->marker_name << " is now at "
                     << feedback->pose.position.x << ", " << feedback->pose.position.y
                     << ", " << feedback->pose.position.z );
}


void HeadPlugin::setEyePose(const geometry_msgs::Pose pose) {

    gazebo::math::Pose p1, p2;
    p1.pos.x = pose.position.x;
    p1.pos.y = pose.position.y;
    p1.pos.z = pose.position.z;
    p1.rot.x = pose.orientation.x;
    p1.rot.y = pose.orientation.y;
    p1.rot.z = pose.orientation.z;
    p1.rot.w = pose.orientation.w;
    p2 = p1;
    p2.pos.x = p1.pos.x + 0.1;

    links_[1]->SetWorldPose(p1);
    links_[2]->SetWorldPose(p2);
    //gazebo::math::Pose p1 = links_[0]->GetWorldPose(), p2 = links_[1]->GetWorldPose();
    //ROS_INFO_STREAM("a: " << links_.size() << ", " << p1 << ", " << p2);
}
