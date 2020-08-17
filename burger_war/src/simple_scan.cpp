#include <boost/optional.hpp>
#include <ctime>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/simple_client_goal_state.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "lib/war_state.hpp"
#include "move_base_msgs/MoveBaseAction.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/console.h"
#include "ros/duration.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "ros/ros.h"
#include "ros/topic.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/buffer_core.h"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

/*
 * 場のマーカーをとにかく取りに行く
 */

const tf2::Transform red_robot_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-1.3, 0, 0));
const tf2::Transform blue_robot_initial_transform(
    tf2::Quaternion(0, 0, 1, 0), tf2::Vector3(1.3, 0, 0));

const tf2::Transform tomato_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.530, 0.530, 0));
const tf2::Transform omelette_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0.530, -0.530, 0));
const tf2::Transform pudding_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-0.530, 0.530, 0));
const tf2::Transform octopus_wiener_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(-0.530, -0.530, 0));
const tf2::Transform fried_shrimp_initial_transform(
    tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, 0));

struct TargetProperty {
  enum class Base { Robot, Obstacle };

  std::string tf_id;
  Base base;

  TargetProperty(const std::string&& tf_id, const Base base)
      : tf_id(tf_id), base(base) {}
};
const std::map<std::string, TargetProperty> target_properties{
    {"RE_L", {"red_left_tag", TargetProperty::Base::Robot}},
    {"RE_R", {"red_right_tag", TargetProperty::Base::Robot}},
    {"RE_B", {"red_back_tag", TargetProperty::Base::Robot}},
    {"BL_L", {"blue_left_tag", TargetProperty::Base::Robot}},
    {"BL_R", {"blue_right_tag", TargetProperty::Base::Robot}},
    {"BL_B", {"blue_back_tag", TargetProperty::Base::Robot}},
    {"Tomato_N", {"tomato_n_tag", TargetProperty::Base::Obstacle}},
    {"Tomato_S", {"tomato_s_tag", TargetProperty::Base::Obstacle}},
    {"Omelette_N", {"omelette_n_tag", TargetProperty::Base::Obstacle}},
    {"Omelette_S", {"omelette_s_tag", TargetProperty::Base::Obstacle}},
    {"Pudding_N", {"pudding_n_tag", TargetProperty::Base::Obstacle}},
    {"Pudding_S", {"pudding_s_tag", TargetProperty::Base::Obstacle}},
    {"OctopusWiener_N",
     {"octopus_wiener_n_tag", TargetProperty::Base::Obstacle}},
    {"OctopusWiener_S",
     {"octopus_wiener_s_tag", TargetProperty::Base::Obstacle}},
    {"FriedShrimp_N", {"fried_shrimp_n_tag", TargetProperty::Base::Obstacle}},
    {"FriedShrimp_S", {"fried_shrimp_s_tag", TargetProperty::Base::Obstacle}},
    {"FriedShrimp_E", {"fried_shrimp_e_tag", TargetProperty::Base::Obstacle}},
    {"FriedShrimp_W", {"fried_shrimp_w_tag", TargetProperty::Base::Obstacle}}};

move_base_msgs::MoveBaseGoal from_transform_to_move_base_goal(
    const std::string& frame, const tf2::Transform& t) {
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = frame;
  goal.target_pose.pose.position.x = t.getOrigin().x();
  goal.target_pose.pose.position.y = t.getOrigin().y();
  goal.target_pose.pose.position.z = t.getOrigin().z();
  goal.target_pose.pose.orientation.x = t.getRotation().x();
  goal.target_pose.pose.orientation.y = t.getRotation().y();
  goal.target_pose.pose.orientation.z = t.getRotation().z();
  goal.target_pose.pose.orientation.w = t.getRotation().w();
  return goal;
}

tf2::Transform get_tf2_transform(
    const std::string& target_frame,
    const std::string& source_frame,
    const tf2_ros::Buffer& buffer,
    const ros::Time& time,
    const ros::Duration& duration) {
  tf2::Transform transform;
  tf2::convert(
      buffer.lookupTransform(target_frame, source_frame, time, duration)
          .transform,
      transform);
  return transform;
}

// 自分のものになっていない障害物のタグのうち一番近いものを見つける
boost::optional<std::pair<const war_state::Target*, tf2::Transform>>
find_nearest_takeable_target(
    const std::vector<war_state::Target>& targets,
    const tf2::Transform& current_transform,
    const war_state::Player::Side& my_side,
    const tf2_ros::Buffer& transform_buffer) {
  auto showld_skip = [&](const war_state::Target& t) {
    return target_properties.at(t.name).base == TargetProperty::Base::Robot ||
           t.owner == static_cast<war_state::Target::Owner>(my_side);
  };
  auto begin = targets.begin();
  const auto end = targets.end();
  while (begin != end && showld_skip(*begin)) {
    ++begin;
  }
  if (begin == end) {
    return boost::none;
  }
  auto best = begin;
  auto cursor = begin;
  try {
    tf2::Transform best_transform = get_tf2_transform(
        "map",
        target_properties.at(best->name).tf_id,
        transform_buffer,
        ros::Time(0),
        ros::Duration(0));

    const auto planar_norm2 = [](const tf2::Transform& a,
                                 const tf2::Transform& b) {
      const auto d = a.getOrigin() - b.getOrigin();
      const tf2::Vector3 plane_d{d.x(), d.y(), 0};
      return plane_d.length2();
    };
    while (cursor != end) {
      if (!showld_skip(*cursor)) {
        tf2::Transform cursor_transfrom = get_tf2_transform(
            "map",
            target_properties.at(cursor->name).tf_id,
            transform_buffer,
            ros::Time(0),
            ros::Duration(0));
        if (planar_norm2(cursor_transfrom, current_transform) <
            planar_norm2(best_transform, current_transform)) {
          best_transform = cursor_transfrom;
          best = cursor;
        }
      }
      ++cursor;
    }

    return std::make_pair(&(*best), best_transform);

  } catch (tf2::TransformException e) {
    ROS_WARN_STREAM(__func__ << ": " << e.what());
    return boost::none;
  }
}

// 自分のものになっていない障害物のタグのうち一番近いところをゴールにする
move_base_msgs::MoveBaseGoal decide_next_goal(
    const war_state::State& state,
    const war_state::Player::Side& my_side,
    const tf2::Transform& current_transform,
    const tf2_ros::Buffer& transform_buffer) {
  // ターゲットから5cm離れたところを目標にする
  const auto offset = tf2::Transform(
      tf2::Quaternion(-0.5, 0.5, 0.5, 0.5), tf2::Vector3(0, 0, 0.20));
  tf2::Transform goal_frame_transform;
  const auto target = find_nearest_takeable_target(
      state.targets, current_transform, my_side, transform_buffer);
  if (target) {
    return from_transform_to_move_base_goal(
        "map", target.get().second * offset);
  } else {
    return from_transform_to_move_base_goal("map", current_transform);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simple_scanner");
  ros::NodeHandle n;

  const war_state::Player::Side my_side = [&]() {
    std::string side_str;
    ros::param::get("~side", side_str);
    if (side_str == "r") {
      return war_state::Player::Side::Red;
    }
    if (side_str == "b") {
      return war_state::Player::Side::Blue;
    }

    ROS_ERROR_STREAM("invalid side parameter: " << side_str);
    exit(1);
  }();

  war_state::State state = war_state::State::from_message(
      ros::topic::waitForMessage<std_msgs::String>("war_state")->data);
  const auto state_sub = n.subscribe<std_msgs::String>(
      "war_state", 100, [&](const std_msgs::String::ConstPtr& msg) {
        try {
          state = std::move(war_state::State::from_message(msg->data));
        } catch (war_state::State::InitializationException e) {
          ROS_ERROR_STREAM(e.what());
          exit(1);
        }
      });

  // マップ内の現在位置
  tf2::Transform current_transform = [&]() {
    switch (my_side) {
      case war_state::Player::Side::Red:
        return red_robot_initial_transform;
      case war_state::Player::Side::Blue:
        return blue_robot_initial_transform;
    }
  }();
  const auto current_transform_sub =
      n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
          "pose_with_covariance",
          100,
          [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
            const auto& position = msg->pose.pose.position;
            current_transform.setOrigin({position.x, position.y, position.z});
            const auto& orientation = msg->pose.pose.orientation;
            current_transform.setRotation(
                {orientation.x, orientation.y, orientation.z, orientation.w});
          });

  tf2_ros::Buffer transform_buffer;
  tf2_ros::TransformListener transform_listener(transform_buffer);

  tf2::Transform red_robot_estimated_transform = red_robot_initial_transform;
  tf2::Transform blue_robot_estimated_transform = blue_robot_initial_transform;
  tf2::Transform tomato_transform = tomato_initial_transform;
  tf2::Transform omelette_transform = omelette_initial_transform;
  tf2::Transform pudding_transform = pudding_initial_transform;
  tf2::Transform octopus_wiener_transform = octopus_wiener_initial_transform;
  tf2::Transform fried_shrimp_transform = fried_shrimp_initial_transform;

  const auto broadcast_transform = [&](const std::string& name,
                                       const tf2::Transform t) {
    static tf2_ros::TransformBroadcaster br;

    tf2::Stamped<tf2::Transform> stamped(t, ros::Time::now(), "map");
    geometry_msgs::TransformStamped msg =
        tf2::toMsg(tf2::Stamped<tf2::Transform>(t, ros::Time::now(), "map"));
    msg.child_frame_id = name;
    br.sendTransform(msg);
  };
  const auto broadcast_obstacles = [&]() {
    broadcast_transform(
        "red_robot_estimated_state/base_footprint",
        red_robot_estimated_transform);
    broadcast_transform(
        "blue_robot_estimated_state/base_footprint",
        blue_robot_estimated_transform);
    broadcast_transform("tomato", tomato_transform);
    broadcast_transform("omelette", omelette_transform);
    broadcast_transform("pudding", pudding_transform);
    broadcast_transform("octopus_wiener", octopus_wiener_transform);
    broadcast_transform("fried_shrimp", fried_shrimp_transform);
  };

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      move_base_action_client("move_base", true);
  while (!move_base_action_client.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_action_client.sendGoal(
      decide_next_goal(state, my_side, current_transform, transform_buffer));

  auto rate = ros::Rate(10);
  while (n.ok()) {
    using GoalState = actionlib::SimpleClientGoalState;
    if (move_base_action_client.getState().isDone()) {
      move_base_action_client.sendGoal(decide_next_goal(
          state, my_side, current_transform, transform_buffer));
    }
    broadcast_obstacles();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
