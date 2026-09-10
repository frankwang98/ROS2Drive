#include "planning/reference_path_planner.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace sdc::planning {
namespace { double dist(const domain::Pose2D& a, const domain::Pose2D& b) { return std::hypot(a.x-b.x, a.y-b.y); } }
ReferencePathPlanner::ReferencePathPlanner() = default;
ReferencePathPlanner::ReferencePathPlanner(Config config) : config_(config) {}

PlanningResult ReferencePathPlanner::plan(const PlanningInput& in) {
  PlanningResult out;
  if (!in.vehicle.localized) { out.reason = "vehicle_not_localized"; return out; }
  if (in.reference_path.size() < 2) { out.reason = "reference_path_too_short"; return out; }
  std::size_t nearest = 0; double best = std::numeric_limits<double>::max();
  for (std::size_t i=0; i<in.reference_path.size(); ++i) { const double d=dist(in.vehicle.pose,in.reference_path[i]); if(d<best){best=d;nearest=i;} }
  const bool closed=dist(in.reference_path.front(),in.reference_path.back()) < std::max(1.0,config_.spacing*4.0);
  const std::size_t segment_count=in.reference_path.size()-1;
  double traveled=0.0, time=0.0; std::size_t segment=nearest;
  while(traveled<=config_.horizon) {
    if(segment>=segment_count) { if(!closed) break; segment=0; }
    const auto& a=in.reference_path[segment]; const auto& b=in.reference_path[segment+1]; const double length=dist(a,b);
    const int samples=std::max(1, static_cast<int>(std::ceil(length/config_.spacing)));
    for(int j=(segment==nearest?0:1); j<=samples && traveled<=config_.horizon; ++j) {
      const double t=static_cast<double>(j)/samples; domain::Waypoint p;
      p.pose.x=a.x+(b.x-a.x)*t; p.pose.y=a.y+(b.y-a.y)*t; p.pose.yaw=std::atan2(b.y-a.y,b.x-a.x);
      p.relative_time=time; out.trajectory.points.push_back(p);
      time += config_.spacing/std::max(0.1,in.speed_limit);
    }
    traveled += length; ++segment;
  }
  // 通用教学基线：沿轨迹法向做局部位移。复杂场景应替换为实现同一接口的
  // 优化/采样 planner；这里不包含任何圆环坐标假设。
  for(auto& point:out.trajectory.points) for(const auto& obstacle:in.obstacles) {
    const double clearance=obstacle.radius+config_.obstacle_margin;
    if(dist(point.pose,obstacle.pose)>=clearance) continue;
    const double nx=-std::sin(point.pose.yaw), ny=std::cos(point.pose.yaw);
    const double side=((point.pose.x-obstacle.pose.x)*nx+(point.pose.y-obstacle.pose.y)*ny)>=0.0?1.0:-1.0;
    point.pose.x+=side*nx*clearance; point.pose.y+=side*ny*clearance;
  }
  std::size_t first_blocked=out.trajectory.points.size();
  for(std::size_t i=0;i<out.trajectory.points.size();++i) for(const auto& obstacle:in.obstacles)
    if(dist(out.trajectory.points[i].pose,obstacle.pose)<obstacle.radius+config_.obstacle_margin*0.5) first_blocked=std::min(first_blocked,i);
  if(first_blocked<out.trajectory.points.size()) {
    const std::size_t brake_start=first_blocked>12?first_blocked-12:0;
    for(std::size_t i=brake_start;i<out.trajectory.points.size();++i)
      out.trajectory.points[i].stop_required=true;
  }
  for(std::size_t i=1;i+1<out.trajectory.points.size();++i) {
    auto& previous=out.trajectory.points[i-1];
    auto& point=out.trajectory.points[i];
    auto& next=out.trajectory.points[i+1];
    point.pose.yaw=std::atan2(next.pose.y-previous.pose.y,next.pose.x-previous.pose.x);
    const double ds=std::max(1e-3,dist(previous.pose,next.pose));
    const double yaw_delta=std::atan2(std::sin(next.pose.yaw-previous.pose.yaw),
                                      std::cos(next.pose.yaw-previous.pose.yaw));
    point.curvature=yaw_delta/ds;
  }
  out.trajectory.frame_id="map"; out.trajectory.stamp_s=in.now_s; out.trajectory.valid=!out.trajectory.points.empty();
  out.success=out.trajectory.valid; out.reason=out.success?"ok":"empty_trajectory"; return out;
}
}  // namespace sdc::planning
