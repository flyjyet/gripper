#include "ui/runtime_log_model.hpp"

#include <utility>

namespace gripper::ui {

RuntimeLogModel::RuntimeLogModel(std::size_t capacity) : capacity_(capacity) {}

void RuntimeLogModel::setCapacity(std::size_t capacity) {
  std::lock_guard<std::mutex> lock{mutex_};
  capacity_ = capacity == 0 ? 1 : capacity;
  while (entries_.size() > capacity_) {
    entries_.erase(entries_.begin());
  }
}

void RuntimeLogModel::append(std::string message) {
  std::lock_guard<std::mutex> lock{mutex_};
  entries_.push_back(RuntimeLogEntry{std::move(message)});
  while (entries_.size() > capacity_) {
    entries_.erase(entries_.begin());
  }
}

std::vector<RuntimeLogEntry> RuntimeLogModel::entries() const {
  std::lock_guard<std::mutex> lock{mutex_};
  return entries_;
}

void RuntimeLogModel::clear() {
  std::lock_guard<std::mutex> lock{mutex_};
  entries_.clear();
}

}  // namespace gripper::ui
