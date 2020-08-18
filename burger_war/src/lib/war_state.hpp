#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace war_state {
  struct Player {
    // Target::Ownerにstatic_castをするため値を揃えるよう注意
    enum class Side { Red = 0, Blue = 1 };
    std::string name;
    Side side;

    Player(const std::string&& name, const Side& side);
  };

  void to_json(nlohmann::json&, const Player&);
  void to_json(nlohmann::json&, const Player::Side&);

  struct Target {
    // Player::Sideからstatic_castをするため値を揃えるよう注意
    enum class Owner { Red = 0, Blue = 1, None };

    std::string name;
    unsigned int point;
    Owner owner;

    Target(
        const std::string&& name,
        const unsigned int point,
        const Owner& owner = Owner::None);
  };

  void to_json(nlohmann::json&, const Target&);
  void to_json(nlohmann::json&, const Target::Owner&);

  class State {
   public:
    Player players[2];
    std::string state;
    std::vector<Target> targets;
    double time;

   private:
    State(
        const Player&& red,
        const Player&& blue,
        const std::string&& state,
        const std::vector<Target>&& targets,
        const double time);

   public:
    struct InitializationException : public std::exception {
      const std::string message;
      InitializationException(const std::string& message) : message(message) {}
    };
    struct InvalidField : public InitializationException {
      const std::string field;
      const std::string desription;
      InvalidField(
          const std::string& message,
          const std::string& field,
          const std::string& desription)
          : InitializationException(message),
            field(field),
            desription(desription) {}
      const char* what() const noexcept override;
    };
    struct JSONException : public InitializationException {
      const nlohmann::json::exception exception;
      JSONException(
          const std::string& message, const nlohmann::json::exception&& e)
          : InitializationException(message), exception(std::move(e)) {}
      const char* what() const noexcept override;
    };

    static State from_message(const std::string& message);

    const Player& player(const Player::Side& side) const;
    int calc_score(const Player::Side& side) const;
  };
  void to_json(nlohmann::json&, const State&);

}  // namespace war_state
