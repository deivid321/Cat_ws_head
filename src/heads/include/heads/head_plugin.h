#ifndef _HEAD_PLUGIN_HH_
#define _HEAD_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

namespace gazebo {

  class HeadPlugin : public ModelPlugin {

    private: 
    ros::NodeHandle *node_;
    ros::Publisher joint_pub_;
    ros::Publisher head_yaw_pub_;
    ros::Publisher head_pitch_pub_;
    ros::Publisher left_eye_yaw_pub_;
    ros::Publisher left_eye_pitch_pub_;
    ros::Publisher right_eye_yaw_pub_;
    ros::Publisher right_eye_pitch_pub_;


    sensor_msgs::JointState joint_msg_;
    tf::TransformBroadcaster broadcaster_;
    geometry_msgs::TransformStamped odom_trans_;

    visualization_msgs::InteractiveMarker head_marker_;
    interactive_markers::InteractiveMarkerServer server_;
    math::Pose static_pose_;
    math::Pose left_eye_static_pose_;


    physics::ModelPtr model_;
    gazebo::physics::Link_V links_;

    event::ConnectionPtr update_;
    physics::JointPtr right_eye_joint_, left_eye_joint_;


    public: 
    HeadPlugin();

    virtual ~HeadPlugin() {
        delete node_;
    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    void Update(const common::UpdateInfo &_info);
    void CreateMarker();

    void statePublisher();
    void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
    void setEyePose(const geometry_msgs::Pose);
    void setMarker();
    void setHead(const geometry_msgs::Pose);
    void setEyes(const geometry_msgs::Pose);

    /// \brief Pointer to the model.




  };


}
#endif
