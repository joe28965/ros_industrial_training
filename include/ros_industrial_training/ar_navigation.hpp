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

#ifndef AR_NAVIGATION_HPP_
#define AR_NAVIGATION_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <move_base_msgs/MoveBaseActionResult.h>

class ARNavigation
{
  public:
    ARNavigation();

protected:
    ros::NodeHandle nh_;

  private:
    void visualizationCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent&);
    bool nearGoal(tf::StampedTransform distance);
    ros::Timer timer_;
    ros::Subscriber subscriber_marker_;
    ros::Subscriber subscriber_goal_;
    ros::Publisher publisher_;
    std::string tf_name_;
    tf::TransformListener listener_;
    tf::TransformBroadcaster broadcaster_;
    bool goal_send_;
};

#endif //AR_NAVIGATION_HPP_