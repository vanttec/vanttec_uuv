/** ----------------------------------------------------------------------------
 * @file: tf2_broadcaster.cpp
 * @date: April 10, 2022
 * @author: Pedro Sanchez
 * @email: pedro.sc.97@gmail.com
 * @author: Sebastian Martinez
 * @email: sebas.martp@gmail.com
 * 
 * @brief: Used to publish the current pose of the simulated UUV and correctly
 *         represent it in RViz.
 * https://github.com/ros-visualization/rviz/issues/597
 * -----------------------------------------------------------------------------
 **/

#include "tf2_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(const std::string& _parent, const std::string& _child)
{
    this->parent_frame = _parent;
    this->child_frame = _child;
}

TfBroadcaster::~TfBroadcaster(){}

void TfBroadcaster::BroadcastTransform(const vanttec_uuv::EtaPose& _pose)
{    
    geometry_msgs::TransformStamped transformStamped;
    
    // From NED to RViz reference frame (x forward, y left, z up): negate y and z
    transformStamped.header.stamp               = ros::Time::now();
    transformStamped.header.frame_id            = this->parent_frame;
    transformStamped.child_frame_id             = this->child_frame;
    transformStamped.transform.translation.x    = _pose.x;
    transformStamped.transform.translation.y    = -_pose.y;
    transformStamped.transform.translation.z    = -_pose.z;

    tf2::Quaternion q;
    q.setRPY(_pose.phi, -_pose.theta, -_pose.psi);
    q.normalize();
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    // std::cout << std::endl;
    // std::cout << transformStamped.transform.rotation.x << std::endl;
    // std::cout << transformStamped.transform.rotation.y << std::endl;
    // std::cout << transformStamped.transform.rotation.z << std::endl;
    // std::cout << transformStamped.transform.rotation.w << std::endl;

    // For NED to ENU: swap x and y and negate z
    // transformStamped.transform.rotation.x = _pose.orientation.y;
    // transformStamped.transform.rotation.y = _pose.orientation.x;
    // transformStamped.transform.rotation.z = -_pose.orientation.z;
    // transformStamped.transform.rotation.w = _pose.orientation.w;

    this->br.sendTransform(transformStamped);

    geometry_msgs::PoseStamped      pose;

    pose.header.stamp       = ros::Time::now();
    pose.header.frame_id    = this->parent_frame;
    pose.pose.position.x    = _pose.x;
    pose.pose.position.y    = -_pose.y;
    pose.pose.position.z    = -_pose.z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    this->path.header.stamp     = ros::Time::now();
    this->path.header.frame_id  = this->parent_frame;
    this->path.poses.push_back(pose);
}