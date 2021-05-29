/*******************************************************************************
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include <cmath>

#include <ros/ros.h>
#include "ros_industrial_training/ar_navigation.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

ARNavigation::ARNavigation()
{
  goal_send_ = false;
  timer_ = nh_.createTimer(ros::Duration(3), &ARNavigation::timerCallback, this);
  subscriber_marker_ = nh_.subscribe<visualization_msgs::Marker>("/visualization_marker", 10, &ARNavigation::visualizationCallback, this);
  subscriber_goal_ = nh_.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 10, &ARNavigation::resultCallback, this);
  publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
}

void ARNavigation::visualizationCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
  tf_name_ = "ar_marker_" + std::to_string(msg->id);

  // start with a transform that moves 0.5m in front of the tag
  tf::Transform marker_to_goal_transform;
  tf::Vector3 v;
  v.setValue(0, 0, 0.5);
  marker_to_goal_transform.setOrigin(v);
  tf::Quaternion q;
  q.setValue(0, 0, 0, 1);
  marker_to_goal_transform.setRotation(q);

  // rotate around x axis (pointing the z axis upward)
  q.setRPY(-M_PI_2, 0, 0);
  marker_to_goal_transform.setRotation(marker_to_goal_transform.operator*(q));

  // rotate around z axis (pointing the x axis toward the tag)
  q.setRPY(0, 0, M_PI_2);
  marker_to_goal_transform.setRotation(marker_to_goal_transform.operator*(q));

  // broadcast the transform, to see the goal as tf marker
  geometry_msgs::TransformStamped broadcasted;
  broadcasted.header.frame_id = tf_name_;
  broadcasted.header.stamp = ros::Time::now();
  broadcasted.child_frame_id = "ar_marker_goal";
  broadcasted.transform.translation.x = marker_to_goal_transform.getOrigin().getX();
  broadcasted.transform.translation.y = marker_to_goal_transform.getOrigin().getY();
  broadcasted.transform.translation.z = marker_to_goal_transform.getOrigin().getZ();
  broadcasted.transform.rotation.x = marker_to_goal_transform.getRotation().getX();
  broadcasted.transform.rotation.y = marker_to_goal_transform.getRotation().getY();
  broadcasted.transform.rotation.z = marker_to_goal_transform.getRotation().getZ();
  broadcasted.transform.rotation.w = marker_to_goal_transform.getRotation().getW();
  broadcaster_.sendTransform(broadcasted);
}

void ARNavigation::timerCallback(const ros::TimerEvent&)
{
  if(!goal_send_)
  {
    tf::StampedTransform transform;
    try{
      // check location of robot compared to ar_marker_goal
      listener_.waitForTransform("base_link", "ar_marker_goal", ros::Time(0), ros::Duration(3.0));
      listener_.lookupTransform("base_link", "ar_marker_goal", ros::Time(0), transform);
      if(!nearGoal(transform))
      {
        // get the transform from our goal to the map
        listener_.waitForTransform("map", "ar_marker_goal", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("map", "ar_marker_goal", ros::Time(0), transform);

        // get the RPY and set the roll and pitch to 0
        tf::Quaternion q = transform.getRotation();
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        q.setRPY(0, 0, yaw);
        transform.setRotation(q);

        // convert to posestamped and publish as goal, with frame_id as ar_tag frame (negating need to calculate map -> ar_tag)
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = transform.getOrigin().getX();
        goal.pose.position.y = transform.getOrigin().getY();
        goal.pose.position.z = transform.getOrigin().getZ();
        goal.pose.orientation.x = transform.getRotation().getX();
        goal.pose.orientation.y = transform.getRotation().getY();
        goal.pose.orientation.z = transform.getRotation().getZ();
        goal.pose.orientation.w = transform.getRotation().getW();
        publisher_.publish(goal);
        ROS_INFO("Goal send.");
        goal_send_ = true;
      }
    }
    catch (tf::TransformException ex){
      ROS_INFO("No AR tag was seen, so no goal created.");
      ros::Duration(1.0).sleep();
    }
  }
}

bool ARNavigation::nearGoal(tf::StampedTransform distance)
{
  if(distance.getOrigin().getX() < 0.1)
  {
    if(distance.getOrigin().getY() < 0.1)
    {
      return true;
    }
  }
  return false;
}

void ARNavigation::resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  if(msg->status.text == "Goal reached.")
  {
    goal_send_ = false;
  }
}

int main(int argc, char **argv)
{
  if (!ros::isInitialized())
  {
    ros::init(argc, argv, "arnavigation");
  }
  ARNavigation ar_tracker;
  ros::Rate rate(15.0);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}