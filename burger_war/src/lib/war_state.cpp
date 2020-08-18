#include "war_state.hpp"

#include <exception>
#include <iterator>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>

namespace war_state {
  using nlohmann::json;

  Player::Player(const std::string&& name, const Side& side)
      : name(name), side(side) {}

  void to_json(json& j, const Player& p) {
    j["name"] = p.name;
    j["side"] = p.side;
  }

  void to_json(json& j, const Player::Side& s) {
    switch (s) {
      case Player::Side::Red:
        j = "r";
        break;
      case Player::Side::Blue:
        j = "b";
        break;
    }
  }

  Target::Target(
      const std::string&& name, const unsigned int point, const Owner& owner)
      : name(name), point(point), owner(owner) {}

  void to_json(nlohmann::json& j, const Target& t) {
    j["name"] = t.name;
    j["point"] = t.point;
    j["owner"] = t.owner;
  }
  void to_json(nlohmann::json& j, const Target::Owner& o) {
    switch (o) {
      case Target::Owner::Red:
        j = "r";
        break;
      case Target::Owner::Blue:
        j = "b";
        break;
      case Target::Owner::None:
        j = "n";
        break;
    }
  }

  const char* State::InvalidField::what() const noexcept {
    static std::string buf = "State::InvalidField, message: " + message;
    return buf.c_str();
  }
  const char* State::JSONException::what() const noexcept {
    static std::string buf = "State::JSONException, message: " + message +
                             ", exception: " + exception.what();
    return buf.c_str();
  }

  State::State(
      const Player&& red,
      const Player&& blue,
      const std::string&& state,
      const std::vector<Target>&& targets,
      const double time)
      : players{red, blue}, state(state), targets(targets), time(time) {}

  State State::from_message(const std::string& message) {
    const auto message_json = json::parse(message);
    try {
      const auto player_red = Player(
          message_json["players"]["r"].get<std::string>(), Player::Side::Red);
      const auto player_blue = Player(
          message_json["players"]["b"].get<std::string>(), Player::Side::Blue);

      const auto state = message_json["state"].get<std::string>();

      std::vector<Target> targets;
      for (const auto& target : message_json["targets"]) {
        const auto name = target["name"].get<std::string>();
        const auto owner = [&]() {
          const auto player = target["player"].get<std::string>();
          if (player == "r") {
            return Target::Owner::Red;
          }
          if (player == "b") {
            return Target::Owner::Blue;
          }
          if (player == "n") {
            return Target::Owner::None;
          }

          std::stringstream ss;
          ss << R"(targets[name == )" << name << R"(].player)";
          throw InvalidField(message, ss.str(), "invalid side");
        }();
        const unsigned int point = [&]() {
          try {
            const auto point = std::stoul(target["point"].get<std::string>());
            if (point > std::numeric_limits<unsigned int>::max()) {
              throw std::runtime_error("limit exceeded");
            }
            return point;
          } catch (std::exception e) {
            std::stringstream ss;
            ss << R"(targets[name == )" << name << R"(].point)";
            throw InvalidField(message, ss.str(), e.what());
          }
        }();

        targets.emplace_back(std::move(name), point, owner);
      }

      const auto time = message_json["time"].get<double>();

      return State(
          std::move(player_red),
          std::move(player_blue),
          std::move(state),
          std::move(targets),
          time);
    } catch (json::exception e) {
      throw JSONException(message, std::move(e));
    }
  }

  const Player& State::player(const Player::Side& side) const {
    return players[static_cast<int>(side)];
  }
  int State::calc_score(const Player::Side& side) const {
    int point = 0;
    for (const auto& target : targets) {
      if (target.owner == static_cast<Target::Owner>(side)) {
        point += target.point;
      }
    }
    return point;
  }

  void to_json(nlohmann::json& j, const State& s) {
    j["players"] = s.players;
    j["state"] = s.state;
    j["targets"] = s.targets;
    j["time"] = s.time;
  }

}  // namespace war_state
