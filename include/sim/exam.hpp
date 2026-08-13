#ifndef SELF_DRIVING_CAR_SIM_EXAM_HPP
#define SELF_DRIVING_CAR_SIM_EXAM_HPP

#include <string>
#include <vector>

namespace sdc {

/// 科目二考试编排器。
///
/// 3 个科目（倒车入库 / 侧方停车 / 直角转弯）都布置在**同一条赛道**
/// （ExamTrackMap）上。考试时小车在同一条道路上依次完成各科目：
/// 每完成一个 → 面板提示「完成」→ 自动进入下一个科目；全部完成判定「考试合格」。
///
/// 与旧的"每科目一张地图、靠切地图推进"不同，本版本在同一张地图内
/// 按站点推进，不再切换地图。
class ExamManager {
 public:
  enum class State {
    kIdle,        // 未开始
    kRunning,     // 考试中
    kItemPassed,  // 当前科目刚完成（下一帧推进到下一科目）
    kFinished,    // 全部完成
  };

  ExamManager();

  /// 开始考试：置为 Running，定位到第一个科目（倒车入库）。
  void start();

  /// 当前是否在考试中。
  bool running() const { return state_ == State::kRunning || state_ == State::kItemPassed; }

  /// 全部完成。
  bool finished() const { return state_ == State::kFinished; }

  /// 当前科目序号（0-based，对应 ExamTrackMap 站点），无则 -1。
  int current_index() const { return current_index_; }

  /// 科目总数。
  size_t total() const { return items_.size(); }

  /// 当前科目名称。
  std::string current_name() const;

  /// 通知「当前科目已完成」，内部推进到下一项或结束。
  void on_item_passed();

  /// 人类可读的状态字符串（HUD/话题用）。
  std::string status_text() const;

  /// 是否需要把赛道定位到当前科目（消费式，调用一次后置 false）。
  /// 用于开始考试 / 刚完成一个科目时，把 ExamTrackMap 定位到该科目航点起点。
  bool consume_seek_needed();

 private:
  struct Item {
    std::string name;
  };

  std::vector<Item> items_;
  State state_{State::kIdle};
  int current_index_{-1};
  bool seek_needed_{false};
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_SIM_EXAM_HPP
