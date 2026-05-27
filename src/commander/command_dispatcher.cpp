#include "commander/command_dispatcher.hpp"

#include <utility>

namespace gripper::commander {

void CommandDispatcher::registerCommand(std::string name, Handler handler) {
  handlers_[std::move(name)] = std::move(handler);
}

int CommandDispatcher::dispatch(const std::vector<std::string>& args) const {
  if (args.empty()) {
    return 1;
  }
  const auto iter = handlers_.find(args.front());
  if (iter == handlers_.end()) {
    return 1;
  }
  return iter->second(args);
}

bool CommandDispatcher::hasCommand(const std::string& name) const {
  return handlers_.find(name) != handlers_.end();
}

}  // namespace gripper::commander
