#pragma once

#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

namespace gripper::ui {

struct RuntimeLogEntry {
  std::string message{};
};

class RuntimeLogModel {
 public:
  explicit RuntimeLogModel(std::size_t capacity = 1000);

  void setCapacity(std::size_t capacity);
  void append(std::string message);
  [[nodiscard]] std::vector<RuntimeLogEntry> entries() const;
  void clear();

 private:
  std::size_t capacity_{1000};
  std::vector<RuntimeLogEntry> entries_{};
  mutable std::mutex mutex_{};
};

}  // namespace gripper::ui
