#include "commander/command_line.hpp"

namespace gripper::commander {

std::vector<std::string> collectArgs(int argc, char** argv) {
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    args.emplace_back(argv[i]);
  }
  return args;
}

}  // namespace gripper::commander
