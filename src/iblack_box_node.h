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

#ifndef IBLACK_BOX_NODE_H
#define IBLACK_BOX_NODE_H

#include "blackbox_control/Import.h"
#include "blackbox_control/GetAvailableRange.h"
#include "blackbox_control/Playback.h"
#include "blackbox_control/Export.h"

namespace blackbox_control
{
/// Interface class for the BlackBoxNode for easier testing
class IBlackBoxNode
{
public:
  /// Imports recordings from the ~/fglicense/files/black_box_import.zip file.
  /** @param req The request - not used
   * @param res The response - not used
   */
  virtual bool import_(Import::Request& req, Import::Response& res) = 0;
  /// Exports a selected range of recordings.
  /** @param req The request, that contains two fields: <br>
   *    int32 begin - The beginning timestamp, from which the export should start,
   *    int32 end - The end timestamp, after which no data should be exported.
   * @param res The response, that contains one field: <br>
   *    string file_name - The filename of the exported data. <br>
   *                       The filename is generated automatically based on the current time.
   */
  virtual bool export_(Export::Request& req, Export::Response& res) = 0;
  /// Starts or stops the playback of already recorded data.
  /** @param req The request, that contains two fields: <br>
   *    bool play - if true, the playback starts, if false, it is stopped<br>
   *    int32 pos - selects the datetime to start from using a Unix timestamp
   * @param res The response to be sent after executing the request - Empty.
   */
  virtual bool playback_(Playback::Request& req, Playback::Response& res) = 0;
  /// Returns the smallest and largest timestamp available in the storage.
  /** @param req - The request - not used
   * @param res - The response, that contains two fields: <br>
   *    string begin - The smallest timestamp <br>
   *    string end - The largest timestamp
   */
  virtual bool getAvailableRange_(GetAvailableRange::Request& req, GetAvailableRange::Response& res) = 0;
};
}  // namespace blackbox_control

#endif