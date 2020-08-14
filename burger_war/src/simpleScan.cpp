#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ctime>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

void state_callback(const std_msgs::String::ConstPtr& msg) {}

// 場のマーカーをとにかく取りに行く
int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_scanner");
  ros::NodeHandle n;
  auto state_sub = n.subscribe("war_state", 100, state_callback);
  ros::spin();

  return 0;
}
