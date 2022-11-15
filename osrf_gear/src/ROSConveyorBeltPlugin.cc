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
#include "ROSConveyorBeltPlugin.hh"

//#include "osrf_gear/ConveyorBeltState.h"
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSConveyorBeltPlugin)

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::ROSConveyorBeltPlugin()
{
}

/////////////////////////////////////////////////
ROSConveyorBeltPlugin::~ROSConveyorBeltPlugin()
{
  //this->rosnode_->shutdown();
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  auto rcl_context = rclcpp::contexts::get_global_default_context()->get_rcl_context();
  if (rcl_context) {
      rcl_arguments_t rcl_args = rcl_get_zero_initialized_arguments();

      // Initialize arguments with empty contents
      rcl_parse_arguments(0,NULL, rcl_get_default_allocator(), &rcl_args);

      // Free up space from previous call
      rcl_arguments_fini(&rcl_context->global_arguments);

      rcl_context->global_arguments = rcl_args;
  }

  // Initialize ROS node
  this->ros_node_ = gazebo_ros::Node::Get(_sdf);
  const gazebo_ros::QoS & qos = this->ros_node_->get_qos();

  std::string controlTopic = "conveyor/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "conveyor/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

   // Initialize publisher
  this->pub_ = this->ros_node_->create_publisher<ariac_msgs::msg::ConveyorBeltState>(
    stateTopic, qos.get_publisher_qos(stateTopic, rclcpp::QoS(1)));

  
  this->controlService_ = this->ros_node_->create_service<ariac_msgs::srv::ConveyorBeltControl>(
    controlTopic,
    std::bind(
      &ROSConveyorBeltPlugin::OnControlCommand, this,
      std::placeholders::_1, std::placeholders::_2));
  


  /*
  // load parameters
  this->robotNamespace_ = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    this->robotNamespace_ = _sdf->GetElement(
        "robot_namespace")->Get<std::string>() + "/";
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  std::string controlTopic = "conveyor/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "conveyor/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  ConveyorBeltPlugin::Load(_parent, _sdf);

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace_);

  this->controlService_ = this->rosnode_->advertiseService(controlTopic,
    &ROSConveyorBeltPlugin::OnControlCommand, this);

  // Message used for publishing the state of the conveyor.
  this->statePub = this->rosnode_->advertise<
    osrf_gear::ConveyorBeltState>(stateTopic, 1000);
    */
}

/////////////////////////////////////////////////
void ROSConveyorBeltPlugin::Publish() const
{
  /*
  osrf_gear::ConveyorBeltState stateMsg;
  stateMsg.enabled = this->IsEnabled();
  stateMsg.power = this->Power();
  this->statePub.publish(stateMsg);
  */
}

  
/////////////////////////////////////////////////
bool ROSConveyorBeltPlugin::OnControlCommand(
  ariac_msgs::srv::ConveyorBeltControl::Request::SharedPtr _req,
  ariac_msgs::srv::ConveyorBeltControl::Response::SharedPtr _res)
{
  
  //const osrf_gear::ConveyorBeltControl::Request& req = event.getRequest();
  //osrf_gear::ConveyorBeltControl::Response& res = event.getResponse();
  gzdbg << "Conveyor control service called with: " << _req->power << std::endl;

  //const std::string& callerName = event.getCallerName();
  //gzdbg << "Conveyor control service called by: " << callerName << std::endl;

  // During the competition, this environment variable will be set.
  //bool is_ariac_task_manager = callerName.compare("/gazebo") == 0;
  // compRunning = std::getenv("ARIAC_COMPETITION");
  /*
  if (compRunning && !is_ariac_task_manager)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  } 

  if (is_ariac_task_manager && !this->IsEnabled())
  {
    ROS_WARN_STREAM("Force enabling conveyor because power set by task manager.");
    this->enabled = true;
  }
*/
  if (!this->IsEnabled())
  {
    std::string errStr = "Belt is not currently enabled so power cannot be set. It may be congested.";
    gzerr << errStr << std::endl;
    RCLCPP_ERROR(this->ros_node_->get_logger(), "%s", errStr.c_str());
    _res->success = false;
    return true;
  }

  if (!(0 == _req->power || (_req->power >= 50 && _req->power <= 100)))
  {
    std::string errStr = "Requested belt power is invalid. Accepted values are 0 or in the range [50, 100].";
    gzerr << errStr << std::endl;
    RCLCPP_ERROR(this->ros_node_->get_logger(), "%s", errStr.c_str());
    _res->success = false;
    return true;
  }

  this->SetPower(_req->power);
  _res->success = true;
  
  return true;
}
