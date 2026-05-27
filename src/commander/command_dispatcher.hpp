#pragma once

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace gripper::commander {

class CommandDispatcher {
 public:
  using Handler = std::function<int(const std::vector<std::string>& args)>;

  void registerCommand(std::string name, Handler handler);
  [[nodiscard]] int dispatch(const std::vector<std::string>& args) const;
  [[nodiscard]] bool hasCommand(const std::string& name) const;

 private:
  std::unordered_map<std::string, Handler> handlers_{};
};

}  // namespace gripper::commander
