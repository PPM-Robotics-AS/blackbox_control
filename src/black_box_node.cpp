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

#include <thread>
#include <iostream>
#include <experimental/filesystem>
#include "dirent.h"
#include "std_msgs/Int32.h"
#include "black_box_node.h"

using namespace std;
using namespace blackbox_control;
using namespace ros;
namespace fs = experimental::filesystem;

BlackBoxNode::BlackBoxNode() : home_dir_(getenv("HOME"))
{
  p_exec = [&](const char* cmd) { return exec(cmd); };
  p_system = [&](const char* cmd) { return system(cmd); };
  p_time = [&](void) { return (long int)time(0); };
  p_get_files = [&](string path) { return getFiles(path); };
}

const string BlackBoxNode::NODE_NAME = "/black_box/";
string BAG_FOLDER = "bags";

string BlackBoxNode::getFilePath(string name)
{
  return home_dir_ + NODE_NAME + name;
}

void BlackBoxNode::log(const string& message)
{
  ROS_WARN("%s", message.c_str());
}

string BlackBoxNode::exec(const char* cmd)
{
  array<char, 128> buffer;
  string result;
  shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
  if (!pipe)
    throw runtime_error("popen() failed!");
  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != nullptr)
      result += buffer.data();
  }
  return result;
}

bool BlackBoxNode::playback_(Playback::Request& req, Playback::Response& res)
{
  p_system("killall rosbag");
  p_system("pkill -f time_publisher.sh");
  if (req.play)
  {
    char buff[200];
    snprintf(buff, sizeof(buff), "roslaunch blackbox_control rosbag.launch BAGS:=%s/* START_POS:=%d &", getFilePath(BAG_FOLDER).c_str(), req.pos);
    p_system(buff);
  }
  
  return true;
}

vector<string> BlackBoxNode::getFiles(string path)
{
  auto dirp = opendir(path.c_str());
  dirent* dp;
  vector<string> files;

  if (!dirp)
    return files;

  while ((dp = readdir(dirp)) != NULL)
  {
    string name = dp->d_name;
    if (name != "." && name != "..")
    {
      // log(name);
      files.push_back(name);
    }
  }
  std::sort(files.begin(), files.end());

  (void)closedir(dirp);

  return files;
}

int BlackBoxNode::parseDate(string fileName)  // format: bb_2017-11-24-18-11-43.bag
{
  fileName = fileName.substr(3, 19);  // cut off bb_, index and extension
  tm t = {};
  istringstream ss(fileName);
  ss >> get_time(&t, "%Y-%m-%d-%H-%M-%S");
  return static_cast<long int>(mktime(&t));
}

string BlackBoxNode::exportFileName()
{
  char buff[100000];
  snprintf(buff, sizeof(buff), "black_box_export_%ld.zip", p_time());
  return string(buff);
}

bool BlackBoxNode::export_(Export::Request& req, Export::Response& res)
{
  auto files = p_get_files(getFilePath(BAG_FOLDER));
  string filtered_files;
  for (int i = 0; i < (int)files.size(); i++)
  {
    int file_date = parseDate(files[i]);
    if (file_date >= req.begin && file_date <= req.end)
    {
      filtered_files += files[i] + " ";
    }
  }

  string file_name = exportFileName();
  if (filtered_files.size() > 0)
  {
    char buff[100000];

    snprintf(buff, sizeof(buff), "bash -c 'cd %s;zip -FS ~/black_box/archive/%s %s'", getFilePath(BAG_FOLDER).c_str(),
             file_name.c_str(), filtered_files.c_str());
    p_system(buff);
    
    res.file_name = file_name;
  }

  return true;
}

bool BlackBoxNode::import_(Import::Request& req, Import::Response& res)
{
    char buff[100000];
    snprintf(buff, sizeof(buff), "bash -c 'cd %s;rm *;unzip ~/black_box/archive/black_box_import.zip'",
             getFilePath(BAG_FOLDER).c_str());
    // log(buff);
    p_system(buff);

  return true;
}

bool BlackBoxNode::getAvailableRange_(GetAvailableRange::Request& req, GetAvailableRange::Response& res)
{
  auto files = p_get_files(getFilePath(BAG_FOLDER));
  // log(files.front());
  // log(files.back());
  // cut only the time part
  if (files.size() > 0)
  {
    res.begin = parseDate(files.front());
    res.end = parseDate(files.back());
  }

  return true;
}

void BlackBoxNode::init(std::shared_ptr<IRosProxy> rosProxy)
{
  rosProxy->advertiseService(NODE_NAME + "import", &IBlackBoxNode::import_, this);
  rosProxy->advertiseService(NODE_NAME + "export", &IBlackBoxNode::export_, this);
  rosProxy->advertiseService(NODE_NAME + "playback", &IBlackBoxNode::playback_, this);
  rosProxy->advertiseService(NODE_NAME + "get_available_range", &IBlackBoxNode::getAvailableRange_, this);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "black_box_node");
  auto rp = shared_ptr<RosProxy>(new RosProxy);

  BlackBoxNode bbn;
  bbn.init(rp);

  bbn.showLogo();
  bbn.log("PPM Black Box Node Started");

  ros::spin();

  return 0;
}

void BlackBoxNode::showLogo()
{
  cout << std::endl;
  cout << "***************************************************************************" << std::endl;
  cout << "*                                                                         *" << std::endl;
  cout << "*                          PPM Black Box - by PPM AS                      *" << std::endl;
  cout << "*                                                                         *" << std::endl;
  cout << "***************************************************************************" << std::endl << std::endl;
}

inline bool BlackBoxNode::endsWith(std::string const& value, std::string const& ending)
{
  if (ending.size() > value.size())
    return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}
