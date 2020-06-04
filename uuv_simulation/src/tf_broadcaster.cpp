#include "tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(const std::string& _parent, const std::string& _child)
{
    this->parent_frame = _parent;
    this->child_frame = _child;
}

TfBroadcaster::~TfBroadcaster(){}

void TfBroadcaster::BroadcastTransform(const geometry_msgs::Pose& _pose)
{    
    geometry_msgs::TransformStamped transformStamped;
    
    transformStamped.header.stamp               = ros::Time::now();
    transformStamped.header.frame_id            = this->parent_frame;
    transformStamped.child_frame_id             = this->child_frame;
    transformStamped.transform.translation.x    = _pose.position.x;
    transformStamped.transform.translation.y    = _pose.position.y;
    transformStamped.transform.translation.z    = -_pose.position.z;

    tf2::Quaternion q;
    q.setRPY(0, 0, _pose.orientation.z);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    this->br.sendTransform(transformStamped);

    geometry_msgs::PoseStamped      pose;

    pose.header.stamp       = ros::Time::now();
    pose.header.frame_id    = this->parent_frame;
    pose.pose.position.x    = _pose.position.x;
    pose.pose.position.y    = _pose.position.y;
    pose.pose.position.z    = -_pose.position.z;

    this->path.header.stamp     = ros::Time::now();
    this->path.header.frame_id  = this->parent_frame;
    this->path.poses.push_back(pose);
}