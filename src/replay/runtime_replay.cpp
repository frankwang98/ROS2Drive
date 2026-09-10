#include "replay/runtime_replay.hpp"

#include <fstream>
#include <iomanip>
#include <utility>

namespace sdc::replay {
void RuntimeRecorder::record(ReplayFrame frame) { frames_.push_back(std::move(frame)); }

bool RuntimeRecorder::save(const std::string& path) const {
  std::ofstream stream(path); if(!stream) return false;
  stream << "SDC_REPLAY_V1\n" << frames_.size() << '\n' << std::setprecision(17);
  for(const auto& frame:frames_) {
    const auto& v=frame.vehicle;
    stream<<frame.timestamp_s<<' '<<v.pose.x<<' '<<v.pose.y<<' '<<v.pose.yaw<<' '
          <<v.velocity.linear<<' '<<v.velocity.angular<<' '<<v.steering_angle<<' '
          <<v.stamp_s<<' '<<v.localized<<' '<<frame.obstacles.size()<<' ';
    for(const auto& obstacle:frame.obstacles)
      stream<<std::quoted(obstacle.id)<<' '<<obstacle.pose.x<<' '<<obstacle.pose.y<<' '
            <<obstacle.pose.yaw<<' '<<obstacle.radius<<' '<<obstacle.dynamic<<' ';
    const auto& output=frame.expected_output;
    stream<<static_cast<int>(output.state)<<' '<<output.control.target_speed<<' '
          <<output.control.steering_angle<<' '<<output.control.brake<<' '
          <<output.control.emergency_stop<<' '
          <<output.planning.trajectory.points.size()<<' ';
    for(const auto& point:output.planning.trajectory.points)
      stream<<point.pose.x<<' '<<point.pose.y<<' '<<point.pose.yaw<<' '
            <<point.curvature<<' '<<point.velocity<<' '<<point.acceleration<<' '
            <<point.relative_time<<' '<<point.stop_required<<' ';
    stream<<'\n';
  }
  return static_cast<bool>(stream);
}

bool RuntimeRecorder::load(const std::string& path) {
  std::ifstream stream(path); if(!stream) return false;
  std::string magic; std::getline(stream,magic); if(magic!="SDC_REPLAY_V1") return false;
  std::size_t frame_count=0; if(!(stream>>frame_count)) return false;
  std::vector<ReplayFrame> loaded; loaded.reserve(frame_count);
  for(std::size_t frame_index=0;frame_index<frame_count;++frame_index) {
    ReplayFrame frame; std::size_t obstacle_count=0,point_count=0; int state=0;
    auto& v=frame.vehicle;
    if(!(stream>>frame.timestamp_s>>v.pose.x>>v.pose.y>>v.pose.yaw
         >>v.velocity.linear>>v.velocity.angular>>v.steering_angle>>v.stamp_s
         >>v.localized>>obstacle_count)) return false;
    for(std::size_t i=0;i<obstacle_count;++i){domain::Obstacle obstacle;if(!(stream>>std::quoted(obstacle.id)>>obstacle.pose.x>>obstacle.pose.y>>obstacle.pose.yaw>>obstacle.radius>>obstacle.dynamic))return false;frame.obstacles.push_back(std::move(obstacle));}
    auto& output=frame.expected_output;
    if(!(stream>>state>>output.control.target_speed>>output.control.steering_angle
         >>output.control.brake>>output.control.emergency_stop>>point_count)) return false;
    output.state=static_cast<domain::RuntimeState>(state);
    for(std::size_t i=0;i<point_count;++i){domain::Waypoint point;if(!(stream>>point.pose.x>>point.pose.y>>point.pose.yaw>>point.curvature>>point.velocity>>point.acceleration>>point.relative_time>>point.stop_required))return false;output.planning.trajectory.points.push_back(point);}
    output.planning.trajectory.valid=!output.planning.trajectory.points.empty();
    output.planning.success=output.planning.trajectory.valid;
    loaded.push_back(std::move(frame));
  }
  frames_=std::move(loaded); return true;
}

runtime::RuntimeOutput replayFrame(runtime::VehicleRuntime& runtime,
                                   const ReplayFrame& frame,double dt_s) {
  runtime.updateVehicleState(frame.vehicle);
  runtime.updateObstacles(frame.obstacles,frame.timestamp_s);
  return runtime.step(frame.timestamp_s,dt_s);
}
}  // namespace sdc::replay
