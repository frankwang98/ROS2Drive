#ifndef SELF_DRIVING_CAR_SIM_EXAM_HPP
#define SELF_DRIVING_CAR_SIM_EXAM_HPP

#include <string>
#include <vector>

#include "sim/map.hpp"

namespace sdc {

/// 科目二考试编排器。
///
/// 维护一个项目（地图）队列，按顺序让小车执行各科目；每完成一个切换下一个；
/// 全部完成则判定「考试合格」。每个科目对应一张 ScenarioMap。
class ExamManager {
 public:
  enum class State {
    kIdle,        // 未开始
    kRunning,     // 考试中
    kItemPassed,  // 当前科目刚完成（下一帧切下一项）
    kFinished,    // 全部完成
  };

  ExamManager();

  /// 开始考试：重置队列进度，置为 Running。
  void start();

  /// 当前是否在考试中。
  bool running() const { return state_ == State::kRunning || state_ == State::kItemPassed; }

  /// 全部完成。
  bool finished() const { return state_ == State::kFinished; }

  /// 当前科目序号（0-based），无则 -1。
  int current_index() const { return current_index_; }

  /// 科目总数。
  size_t total() const { return items_.size(); }

  /// 当前科目对应的地图类型。
  MapType current_map_type() const;

  /// 当前科目名称。
  std::string current_name() const;

  /// 通知「当前科目已完成」，内部推进到下一项或结束。
  void on_item_passed();

  /// 人类可读的状态字符串（HUD/话题用）。
  std::string status_text() const;

  /// 是否刚完成一个科目、需要切换地图（消费式，调用一次后置 false）。
  bool consume_switch_needed();

 private:
  struct Item {
    MapType type;
    std::string name;
  };

  std::vector<Item> items_;
  State state_{State::kIdle};
  int current_index_{-1};
  bool switch_needed_{false};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_EXAM_HPP
