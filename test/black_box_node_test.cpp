#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros_mock.h"
#include "../src/ros_proxy.h"
#include "../src/black_box_node.h"
#include "std_msgs/Int32.h"

using namespace blackbox_control;
using ::testing::_;
using ::testing::AtLeast;
using namespace ros;
using ::testing::PrintToString;
using ::testing::Property;
using ::testing::Truly;
using namespace std;

string home_dir = getenv("HOME");

/// Tests, if the home directory is set correctly
TEST(PathTest, homeDirCorrect)
{
  BlackBoxNode bbn;
  EXPECT_EQ(home_dir, bbn.home_dir_);
}

/// Tests, if the getFilePath returns the currect path
TEST(PathTest, returnsCorrectPath1)
{
  BlackBoxNode bbn;
  string a = "test";
  EXPECT_EQ(home_dir + BlackBoxNode::NODE_NAME + a, bbn.getFilePath(a));
}

/// Tests, if the stop playback stops the playback
TEST(PlaybackTest, StopPlayback)
{
  BlackBoxNode bbn;
  vector<string> systemCalls;
  bbn.p_system = [&](const char *cmd) {
    systemCalls.push_back(cmd);
    return 0;
  };

  Playback::Request req;
  Playback::Response res;

  req.play = false;

  bbn.playback_(req, res);

  EXPECT_EQ("killall rosbag", systemCalls.at(0));
  EXPECT_EQ("pkill -f time_publisher.sh", systemCalls.at(1));
  EXPECT_EQ(systemCalls.size(), 2);
}

/// Tests, if the start playback starts the playback and the values are set correctly
TEST(PlaybackTest, StartPlayback)
{
  BlackBoxNode bbn;
  vector<string> systemCalls;
  bbn.p_system = [&](const char *cmd) {
    systemCalls.push_back(cmd);
    return 0;
  };

  Playback::Request req;
  Playback::Response res;

  req.play = true;
  req.pos = 5;

  bbn.playback_(req, res);

  EXPECT_EQ("killall rosbag", systemCalls.at(0));
  EXPECT_EQ("pkill -f time_publisher.sh", systemCalls.at(1));
  EXPECT_EQ("roslaunch blackbox_control rosbag.launch BAGS:=" + bbn.getFilePath("bags") + "/* START_POS:=5 &", systemCalls.at(2));
  EXPECT_EQ(systemCalls.size(), 3);
}

/// Tests, if the getFiles returns a list of files by using the / dir
TEST(GetFilesTest, ReturnsListOfFiles)
{
  BlackBoxNode bbn;
  vector<string> files = bbn.getFiles("/");
  EXPECT_GT(files.size(), 1);
  // check the ordering
  for (uint i = 1; i < files.size(); i++)
    EXPECT_GT(files.at(i), files.at(i - 1));
}

/// Tests, if the date is parsed correctly out of filenames
TEST(ParseDate, ParsesDates)
{
  BlackBoxNode bbn;

  EXPECT_EQ(1511543503, bbn.parseDate("bb_2017-11-24-18-11-43.bag"));
  EXPECT_EQ(1553173953, bbn.parseDate("bb_2019-03-21-14-12-33.bag"));
  EXPECT_EQ(1659312123, bbn.parseDate("bb_2022-08-01-01-02-03.bag"));
}

/// Tests, if the correct filename is generated using a specific timestamp
TEST(ExportFileName, GeneratesCorrectName)
{
  BlackBoxNode bbn;
  int t = 25;
  bbn.p_time = [&](void) { return t; };
  EXPECT_EQ("black_box_export_25.zip", bbn.exportFileName());
  t = 5435;
  EXPECT_EQ("black_box_export_5435.zip", bbn.exportFileName());
  t = 65423;
  EXPECT_EQ("black_box_export_65423.zip", bbn.exportFileName());
}

/// Tests, if the file export is started, and all parameters are set correctly
TEST(Export, ExportsFiles)
{
  BlackBoxNode bbn;
  string systemCall = "";
  bbn.p_system = [&](const char *cmd) {
    systemCall = cmd;
    return 0;
  };
  bbn.p_get_files = [&](string path) {
    vector<string> result;
    for (int i = 10; i < 60; i++)
      result.push_back("bb_2017-11-24-18-11-" + PrintToString(i) + ".bag");

    return result;
  };

  int t = 99;
  bbn.p_time = [&](void) { return t; };

  Export::Request req;
  Export::Response res;

  int count = 8;
  req.begin = 1511543503;
  req.end = req.begin + count;
  bbn.export_(req, res);

  int baseLength = home_dir.size() + 108;
  int itemLength = 27;
  EXPECT_EQ(baseLength + itemLength * count, systemCall.length());
  EXPECT_EQ("black_box_export_99.zip", res.file_name);

  count = 10;
  req.end = req.begin + count;
  bbn.export_(req, res);

  EXPECT_EQ(baseLength + itemLength * count, systemCall.length());
}

/// Tests, if the file import is initiated
TEST(Import, ImportsFiles)
{
  BlackBoxNode bbn;
  string systemCall = "";
  bbn.p_system = [&](const char *cmd) {
    systemCall = cmd;
    return 0;
  };
  Import::Request req;
  Import::Response res;
  bbn.import_(req, res);
  EXPECT_NE(systemCall.length(), 0);
}

/// Tests, if the available range is returned correctly using a preset list of files
TEST(AvailableRange, ReturnsAvailableRange)
{
  BlackBoxNode bbn;
  bbn.p_get_files = [&](string path) {
    vector<string> result;
    for (int i = 10; i < 60; i++)
      result.push_back("bb_2017-11-24-18-11-" + PrintToString(i) + ".bag");

    return result;
  };

  GetAvailableRange::Request req;
  GetAvailableRange::Response res;

  bbn.getAvailableRange_(req, res);

  EXPECT_EQ(1511543470, res.begin);
  EXPECT_EQ(1511543519, res.end);
}

/// Tests, if the services are advertised
TEST(Advertise, AdvertisesServices)
{
  BlackBoxNode bbn;
  std::shared_ptr<RosProxyFake> p_rpf(new RosProxyFake);
  bbn.init(p_rpf);
  EXPECT_EQ(4, p_rpf->advertisedServices.size());
  EXPECT_TRUE(std::find(p_rpf->advertisedServices.begin(), p_rpf->advertisedServices.end(), "/black_box/import") != p_rpf->advertisedServices.end());
  EXPECT_TRUE(std::find(p_rpf->advertisedServices.begin(), p_rpf->advertisedServices.end(), "/black_box/export") != p_rpf->advertisedServices.end());
  EXPECT_TRUE(std::find(p_rpf->advertisedServices.begin(), p_rpf->advertisedServices.end(), "/black_box/playback") != p_rpf->advertisedServices.end());
  EXPECT_TRUE(std::find(p_rpf->advertisedServices.begin(), p_rpf->advertisedServices.end(), "/black_box/get_available_range") != p_rpf->advertisedServices.end());
}

/// Runs all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}