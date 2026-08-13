#include "sim/exam.hpp"

namespace sdc {

ExamManager::ExamManager() {
  // 科目二项目队列（顺序可调）
  items_ = {
      {MapType::kReverseParking, "倒车入库"},
      {MapType::kSideParking,    "侧方停车"},
      {MapType::kRightAngleTurn, "直角转弯"},
  };
}

void ExamManager::start() {
  state_ = State::kRunning;
  current_index_ = 0;
  switch_needed_ = true;  // 触发首张地图加载
}

MapType ExamManager::current_map_type() const {
  if (current_index_ < 0 || current_index_ >= static_cast<int>(items_.size()))
    return MapType::kRing;
  return items_[current_index_].type;
}

std::string ExamManager::current_name() const {
  if (current_index_ < 0 || current_index_ >= static_cast<int>(items_.size()))
    return "";
  return items_[current_index_].name;
}

void ExamManager::on_item_passed() {
  if (!running()) return;
  ++current_index_;
  if (current_index_ >= static_cast<int>(items_.size())) {
    state_ = State::kFinished;
    switch_needed_ = false;
  } else {
    state_ = State::kRunning;
    switch_needed_ = true;  // 切到下一张地图
  }
}

bool ExamManager::consume_switch_needed() {
  bool v = switch_needed_;
  switch_needed_ = false;
  return v;
}

std::string ExamManager::status_text() const {
  switch (state_) {
    case State::kIdle:
      return "待开始";
    case State::kRunning: {
      int done = current_index_;  // 已完成数
      if (done < 0) done = 0;
      char buf[128];
      std::snprintf(buf, sizeof(buf), "[%d/%zu] %s 进行中",
                    current_index_ + 1, items_.size(), current_name().c_str());
      return buf;
    }
    case State::kItemPassed:
      return "科目完成，切换中…";
    case State::kFinished:
      return "考试合格！全部科目完成";
  }
  return "";
}

}  // namespace sdc
