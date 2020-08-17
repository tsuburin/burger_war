#include "lib/war_state.hpp"
#include "nlohmann/json.hpp"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "std_msgs/String.h"

// lib/war_state.{cpp, hpp} のテスト用ノード
// inputを審判サーバーからパブリッシュされるwar_stateとして読んで、
// outputにパース結果をjsonで出力する

ros::Publisher pub;
void callback(const std_msgs::String::ConstPtr& msg) {
  const auto state = war_state::State::from_message(msg->data);
  const nlohmann::json state_json = [&]() {
    try {
      return nlohmann::json{state};
    } catch (war_state::State::InitializationException e) {
      ROS_ERROR_STREAM(e.what());
      exit(1);
    }
  }();

  ROS_INFO_STREAM("echo_war_state: " << state_json);
  std_msgs::String msg_to_echo;
  msg_to_echo.data = state_json.dump();
  pub.publish(msg_to_echo);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "echo_war_state");
  ros::NodeHandle n;

  pub = n.advertise<std_msgs::String>("output", 1000);
  auto sub = n.subscribe("input", 1000, callback);
  ros::spin();
}
