#ifndef ROS_MOCK_H
#define ROS_MOCK_H

#include "gmock/gmock.h"  // Brings in Google Mock.
#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "../src/ros_proxy.h"
#include "vector"

using namespace ros;
using namespace std;

namespace blackbox_control
{
/// Mock class to monitor function calls of the IPublisher class
class MockPublisher : public IPublisher
{
public:
  /// Mock method to monitor function calls of the publish function
  MOCK_METHOD1(publish, void(std_msgs::Int32 msg));
};

/// Fake class to monitor function calls of the IRosProxy class
class RosProxyFake : public IRosProxy
{
public:
  /// Pointer to the MockPublisher
  shared_ptr<MockPublisher> publisher = shared_ptr<MockPublisher>(new MockPublisher);

  /// Faked advertise function
  /** @param path The path of the advertised topic
   */
  shared_ptr<IPublisher> advertise(string path)
  {
    return publisher;
  }

  /// Vector of all advertised service paths, to be checked after calling the advertiseServices
  vector<string> advertisedServices;

  /// Faked advertiseService function
  /** Only adds the service param to the advertisedServices member for a check later
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Import::Request& req,
                                                             blackbox_control::Import::Response& res),
                                IBlackBoxNode* obj)
  {
    advertisedServices.push_back(service);
  }

  /// Faked advertiseService function
  /** Only adds the service param to the advertisedServices member for a check later
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Export::Request& req,
                                                             blackbox_control::Export::Response& res),
                                IBlackBoxNode* obj)
  {
    advertisedServices.push_back(service);
  }
  /// Faked advertiseService function
  /** Only adds the service param to the advertisedServices member for a check later
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::Playback::Request& req,
                                                             blackbox_control::Playback::Response& res),
                                IBlackBoxNode* obj)
  {
    advertisedServices.push_back(service);
  }
  /// Faked advertiseService function
  /** Only adds the service param to the advertisedServices member for a check later
   * @param service The path of the service to be advertised,
   * @param callb The callback function to be called after a service request
   * @param obj The object, upon which the callback function should be called
   */
  virtual void advertiseService(const string& service,
                                bool (IBlackBoxNode::*callb)(blackbox_control::GetAvailableRange::Request& req,
                                                             blackbox_control::GetAvailableRange::Response& res),
                                IBlackBoxNode* obj)
  {
    advertisedServices.push_back(service);
  }
};
}  // namespace blackbox_control
#endif
