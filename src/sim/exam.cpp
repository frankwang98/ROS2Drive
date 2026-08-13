#include "sim/exam.hpp"

#include <cstdio>

namespace sdc {

ExamManager::ExamManager() {
  // 科目二项目队列（顺序可调），全部在同一张赛道地图上按站点推进。
  items_ = {
      {"倒车入库"},
      {"侧方停车"},
      {"直角转弯"},
  };
}

void ExamManager::start() {
  state_ = State::kRunning;
  current_index_ = 0;
  seek_needed_ = true;  // 触发赛道定位到第一个科目
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
    seek_needed_ = false;
  } else {
    state_ = State::kRunning;
    seek_needed_ = true;  // 定位到下一个科目（仍在同一赛道）
  }
}

bool ExamManager::consume_seek_needed() {
  bool v = seek_needed_;
  seek_needed_ = false;
  return v;
}

std::string ExamManager::status_text() const {
  switch (state_) {
    case State::kIdle:
      return "待开始";
    case State::kRunning: {
      char buf[128];
      std::snprintf(buf, sizeof(buf), "[%d/%zu] %s 进行中",
                    current_index_ + 1, items_.size(), current_name().c_str());
      return buf;
    }
    case State::kItemPassed:
      return "科目完成，切换下一科目…";
    case State::kFinished:
      return "考试合格！全部科目完成";
  }
  return "";
}

}  // namespace sdc
