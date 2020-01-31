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

#ifndef BLACK_BOX_NODE_H
#define BLACK_BOX_NODE_H

#include <stdlib.h>
#include <string>
#include <array>
#include <cstdio>
#include "ros/ros.h"
#include "ros_proxy.h"
#include "iblack_box_node.h"
typedef std::string string;

namespace blackbox_control
{
/// Main class to control the record and playback
/** The BlackBoxNode class is responsible for starting and stopping the recording of data,
 * importing and exporting selected ranges of events.
 */
class BlackBoxNode : public IBlackBoxNode
{
public:
  /// Constructor. Initializes member variables, no exception-capable code is run here, that is in the init.
  BlackBoxNode();

  /// Initializes the object: registers the ROS interface, shows the logo, starts the pos publish thread.
  /** @param rosProxy Pointer to the RosProxy
   */
  void init(std::shared_ptr<IRosProxy> rosProxy);

  /// Starts or stops the playback of already recorded data.
  /** @param req The request, that contains two fields: <br>
   *    bool play - if true, the playback starts, if false, it is stopped<br>
   *    int32 pos - selects the datetime to start from using a Unix timestamp
   * @param res The response to be sent after executing the request - Empty.
   */
  bool playback_(Playback::Request& req, Playback::Response& res);

  /// Exports a selected range of recordings.
  /** @param req The request, that contains two fields: <br>
   *    int32 begin - The beginning timestamp, from which the export should start,
   *    int32 end - The end timestamp, after which no data should be exported.
   * @param res The response, that contains one field: <br>
   *    string file_name - The filename of the exported data. <br>
   *                       The filename is generated automatically based on the current time.
   */
  bool export_(Export::Request& req, Export::Response& res);

  /// Imports recordings from the ~/fglicense/files/black_box_import.zip file.
  /** @param req The request - not used
   * @param res The response - not used
   */
  bool import_(Import::Request& req, Import::Response& res);

  /// Returns the smallest and largest timestamp available in the storage.
  /** @param req - The request - not used
   * @param res - The response, that contains two fields: <br>
   *    string begin - The smallest timestamp <br>
   *    string end - The largest timestamp
   */
  bool getAvailableRange_(GetAvailableRange::Request& req, GetAvailableRange::Response& res);

  // protected:
  /// The location of the home dir
  std::string home_dir_;

  /// The position topic publisher
  std::shared_ptr<IPublisher> pos_pub_;

  /// The name of the node
  static const string NODE_NAME;

  /// Function pointer to the exec function
  std::function<string(const char*)> p_exec;

  /// Function pointer to the system function
  std::function<int(const char*)> p_system;

  /// Function pointer to the time function
  std::function<long int(void)> p_time;

  /// Function pointer to the getFiles function
  std::function<std::vector<string>(string)> p_get_files;

  /// The get available range service member
  ros::ServiceServer get_available_range_service;

  /// The playback service member
  ros::ServiceServer playback_service;

  /// The export service member
  ros::ServiceServer export_service;

  /// The import service member
  ros::ServiceServer import_service;

  /// Assembles the full path for a file name
  /** @param name The name of the file
   * @return The full path of the file
   */
  string getFilePath(string name);

  /// Emits a ROS_INFO message based on the message passed
  /** @param message The message to be written
   */
  void log(const string& message);

  /// Executes a shell command
  /** @param cmd The shell command to be executed
   * @return The result of the command
   */
  string exec(const char* cmd);

  /// Returns the list of files in a directory
  /** @param path The path of the directory
   * @return The list of entries in the directory
   */
  std::vector<string> getFiles(string path);

  /// Converts a filename string to a timestamp
  /** @param fileName The filename to be parsed
   * @return The timestamp extracted from the filename
   */
  int parseDate(string fileName);  // format: 2017-11-24-18-11-43.bag

  /// Returns the automatically generated filename
  /** @return The filename
   */
  string exportFileName();

  /// Shows the PPM Logo
  void showLogo();

  /// Checks, if the value ends with the ending parameter.
  /**@param value The value to be checked
   * @param ending The ending to be checked
   * @return True, if the value ends with the ending
   */
  inline bool endsWith(std::string const& value, std::string const& ending);
};
}  // namespace blackbox_control

#endif