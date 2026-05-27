#pragma once

#include <string>
#include <vector>

namespace gripper::commander {

[[nodiscard]] std::vector<std::string> collectArgs(int argc, char** argv);

}  // namespace gripper::commander
