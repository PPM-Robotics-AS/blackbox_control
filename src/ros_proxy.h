/*
BLACKBOX
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/blackbox

Copyright (c) 2019 PPM Robotics AS

This library is part of BLACKBOX project,
the Focused Technical Project BLACKBOX - an automated trigger-based 
reporting, data recording and playback unit for ROS.

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: http://rosin-project.eu
This project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement no. 732287.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef ROS_PROXY_H
#define ROS_PROXY_H

#include "stdlib.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"
#include "iblack_box_node.h"
#include "blackbox_control/Import.h"

namespace blackbox_control
{
/// Interface class for the PublisherProxy for easier testing
class IPublisher
{
public:
  /// Interface for publishing a topic inside the Publisher.
  /** @param msg The message of the topic to be published
   */
  virtual void publish(std_msgs::Int32 msg) = 0;
};

/// Stores a ROS Publisher, and publishes the value of it
class PublisherProxy : public IPublisher
{
  /// The Publisher object used for publishing
  std::shared_ptr<ros::Publisher> publisher_;

public:
  /// Constructor
  /** @param publisher Pointer to the ROS Publisher object
   */
  PublisherProxy(std::shared_ptr<ros::Publisher> publisher)
  {
    publisher_ = publisher;
  }

  /// Publishes the topic inside the Publisher.
  /** @param msg The message of the topic to be published
   */
  virtual void publish(std_msgs::Int32 msg)
  {
    publisher_->publish(msg);
  }
};

/// Interface class for the RosProxy for easier testing
class IRosProxy
{
public:
  /// Advertises a topic with an Int32 message type
  /** @param path The path of the topic
   */
  virtual std::shared_ptr<IPublisher> advertise(std::string path) = 0;
  /// Interface for advertising a service
  /** @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Import::Request& req,
                                                             blackbox_control::Import::Response& res),
                                IBlackBoxNode* obj) = 0;
  /// Interface for advertising a service
  /** @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Export::Request& req,
                                                             blackbox_control::Export::Response& res),
                                IBlackBoxNode* obj) = 0;
  /// Interface for advertising a service
  /** @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Playback::Request& req,
                                                             blackbox_control::Playback::Response& res),
                                IBlackBoxNode* obj) = 0;
  /// Interface for advertising a service
  /** @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::GetAvailableRange::Request& req,
                                                             blackbox_control::GetAvailableRange::Response& res),
                                IBlackBoxNode* obj) = 0;
};

/// Includes ROS-related functionality abstraction
class RosProxy : public IRosProxy
{
  /// ROS NodeHandle member
  ros::NodeHandle node_handle_;
  /// Vector of the services advertised
  std::vector<ros::ServiceServer> services_;

public:
  /// Advertises a topic with an Int32 message type
  /** @param path The path of the topic
   */
  std::shared_ptr<IPublisher> advertise(std::string path)
  {
    return std::shared_ptr<IPublisher>(new PublisherProxy(
        std::shared_ptr<ros::Publisher>(new ros::Publisher(node_handle_.advertise<std_msgs::Int32>(path, 1)))));
  }

  /// Advertises a service
  /** Uses the member node_handle_ to advertise a service, reginstering a callback method on a specific object.
   * The services are kept inside the RosProxy object, in the services_ vector member.
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Import::Request& req,
                                                             blackbox_control::Import::Response& res),
                                IBlackBoxNode* obj)
  {
    services_.push_back(node_handle_.advertiseService(service, callb, obj));
  }
  /// Advertises a service
  /** Uses the member node_handle_ to advertise a service, reginstering a callback method on a specific object.
   * The services are kept inside the RosProxy object, in the services_ vector member.
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Export::Request& req,
                                                             blackbox_control::Export::Response& res),
                                IBlackBoxNode* obj)
  {
    services_.push_back(node_handle_.advertiseService(service, callb, obj));
  }
  /// Advertises a service
  /** Uses the member node_handle_ to advertise a service, reginstering a callback method on a specific object.
   * The services are kept inside the RosProxy object, in the services_ vector member.
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Playback::Request& req,
                                                             blackbox_control::Playback::Response& res),
                                IBlackBoxNode* obj)
  {
    services_.push_back(node_handle_.advertiseService(service, callb, obj));
  }
  /// Advertises a service
  /** Uses the member node_handle_ to advertise a service, reginstering a callback method on a specific object.
   * The services are kept inside the RosProxy object, in the services_ vector member.
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const std::string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::GetAvailableRange::Request& req,
                                                             blackbox_control::GetAvailableRange::Response& res),
                                IBlackBoxNode* obj)
  {
    services_.push_back(node_handle_.advertiseService(service, callb, obj));
  }
};
}  // namespace blackbox_control
#endif
