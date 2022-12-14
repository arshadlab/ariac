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

#include <memory>
#include <string>
#include <sdf/sdf.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include "osrf_gear/ROSVacuumGripperPlugin.hh"
//#include "osrf_gear/VacuumGripperControl.h"
//#include "osrf_gear/VacuumGripperState.h"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSVacuumGripperPlugin class.
  struct ROSVacuumGripperPluginPrivate
  {
    /// A pointer to the GazeboROS node.
    gazebo_ros::Node::SharedPtr ros_node_;

    /// \brief Publishes the state of the gripper.
    public: ros::Publisher statePub;

    /// \brief Receives service calls to control the gripper.
    public:
      rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr controlService_;
  };
}

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROSVacuumGripperPlugin);

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::ROSVacuumGripperPlugin()
  : VacuumGripperPlugin(),
    dataPtr(new ROSVacuumGripperPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSVacuumGripperPlugin::~ROSVacuumGripperPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Load(physics::ModelPtr _parent,
    sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized,"
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Load SDF parameters.
  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
      "robot_namespace")->Get<std::string>() + "/";
  }

  std::string controlTopic = "gripper/control";
  if (_sdf->HasElement("control_topic"))
    controlTopic = _sdf->Get<std::string>("control_topic");

  std::string stateTopic = "gripper/state";
  if (_sdf->HasElement("state_topic"))
    stateTopic = _sdf->Get<std::string>("state_topic");

  VacuumGripperPlugin::Load(_parent, _sdf);

  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  //this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  // Service for controlling the gripper.
  /*
  this->dataPtr->controlService =
    this->dataPtr->rosnode->advertiseService(controlTopic,
      &ROSVacuumGripperPlugin::OnGripperControl, this);
  */

  // Initialize service
  impl_->controlService_ = impl_->ros_node_->create_service<std_srvs::srv::SetBool>(
    controlTopic,
    std::bind(
      &GazeboRosVacuumGripperPrivate::OnGripperControl, impl_.get(),
      std::placeholders::_1, std::placeholders::_2));
      
  // Message used for publishing the state of the gripper.
  this->dataPtr->statePub = this->dataPtr->rosnode->advertise<
    osrf_gear::VacuumGripperState>(stateTopic, 1000);

  impl_->pub_ = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>(
    stateTopic, qos.get_publisher_qos("grasping", rclcpp::QoS(1)));

}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Reset()
{
  VacuumGripperPlugin::Reset();
}

bool ROSVacuumGripperPlugin::OnGripperControl(
  osrf_gear::VacuumGripperControl::Request &_req,
  osrf_gear::VacuumGripperControl::Response &_res)
{
  gzdbg << "Gripper control requested: " << (_req.enable ? "Enable" : "Disable") << std::endl;
  if (_req.enable)
    this->Enable();
  else
    this->Disable();

  _res.success = true;
  return _res.success;
}

/////////////////////////////////////////////////
void ROSVacuumGripperPlugin::Publish() const
{
  osrf_gear::VacuumGripperState msg;
  msg.attached = this->Attached();
  msg.enabled = this->Enabled();
  impl_->pub_.publish(msg);
}
