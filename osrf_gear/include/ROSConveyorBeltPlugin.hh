/*
 * Copyright 2016 Open Source Robotics Foundation
 *
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
 *
*/

#ifndef _ROS_CONVEYOR_BELT_PLUGIN_HH_
#define _ROS_CONVEYOR_BELT_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <sdf/sdf.hh>
#include <gazebo_ros/node.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
// Gazebo
#include "ariac_msgs/msg/conveyor_belt_state.hpp"
#include "ariac_msgs/srv/conveyor_belt_control.hpp"
#include "ConveyorBeltPlugin.hh"


// ROS
//#include <osrf_gear/ConveyorBeltControl.h>
//#include <ros/ros.h>

namespace gazebo
{
  /// \brief ROS implementation of the ConveyorBeltPlugin plugin
  class ROSConveyorBeltPlugin : public ConveyorBeltPlugin
  {
    /// \brief Constructor
    public: ROSConveyorBeltPlugin();

    /// \brief Destructor
    public: virtual ~ROSConveyorBeltPlugin();

    /// \brief Load the plugin.
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element of the plugin.
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    
    /// \brief Receives requests on the conveyor belt's topic.
    /// \param[in] _req The desired state of the conveyor belt.
    /// \param[in] _res If the service succeeded or not.
    public: bool OnControlCommand(
      ariac_msgs::srv::ConveyorBeltControl::Request::SharedPtr _req,
      ariac_msgs::srv::ConveyorBeltControl::Response::SharedPtr _res);
    
    // Documentation inherited.
    private: virtual void Publish() const;

    /// \brief for setting ROS name space
    private: std::string robotNamespace_;

    /// \brief ros node handle
    //private: ros::NodeHandle *rosnode_;
    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// \brief Receives service calls to control the conveyor belt.
    //public: ros::ServiceServer controlService_;


    /// Service for gripper switch
    rclcpp::Service<ariac_msgs::srv::ConveyorBeltControl>::SharedPtr controlService_;

    /// \brief Publishes the state of the conveyor.
    //public: ros::Publisher statePub;
    /// Publisher for gripper action status
    rclcpp::Publisher<ariac_msgs::msg::ConveyorBeltState>::SharedPtr pub_;
  };
}
#endif
