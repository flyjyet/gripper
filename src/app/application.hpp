#pragma once

#include <iosfwd>
#include <vector>
#include <string>

namespace gripper::app {

class Application {
 public:
  int run(const std::vector<std::string>& args, std::istream& input,
          std::ostream& output);
};

}  // namespace gripper::app
