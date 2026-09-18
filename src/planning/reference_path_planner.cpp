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
  // Project onto the nearest segment instead of snapping to a sparse waypoint.
  // This keeps the local horizon continuous on long mining-road segments.
  std::size_t nearest_segment = 0;
  double nearest_t = 0.0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < in.reference_path.size(); ++i) {
    const auto& a = in.reference_path[i];
    const auto& b = in.reference_path[i + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double length_sq = dx * dx + dy * dy;
    const double t = length_sq > 1e-9
                         ? std::clamp(((in.vehicle.pose.x - a.x) * dx +
                                       (in.vehicle.pose.y - a.y) * dy) /
                                          length_sq,
                                      0.0, 1.0)
                         : 0.0;
    const domain::Pose2D projection{a.x + t * dx, a.y + t * dy, 0.0,
                                    a.z + t * (b.z - a.z)};
    const double d = dist(in.vehicle.pose, projection);
    if (d < best) {
      best = d;
      nearest_segment = i;
      nearest_t = t;
    }
  }
  const bool closed=dist(in.reference_path.front(),in.reference_path.back()) < std::max(1.0,config_.spacing*4.0);
  const std::size_t segment_count=in.reference_path.size()-1;
  double traveled=0.0, time=0.0; std::size_t segment=nearest_segment;
  while(traveled<=config_.horizon) {
    if(segment>=segment_count) { if(!closed) break; segment=0; }
    const auto& a=in.reference_path[segment]; const auto& b=in.reference_path[segment+1]; const double length=dist(a,b);
    const int samples=std::max(1, static_cast<int>(std::ceil(length/config_.spacing)));
    const bool first_segment = segment == nearest_segment;
    const double start_t = first_segment ? nearest_t : 0.0;
    for(int j=(first_segment?0:1); j<=samples && traveled<=config_.horizon; ++j) {
      const double t=first_segment
                        ? start_t + (1.0 - start_t) * static_cast<double>(j) / samples
                        : static_cast<double>(j) / samples;
      domain::Waypoint p;
      p.pose.x=a.x+(b.x-a.x)*t; p.pose.y=a.y+(b.y-a.y)*t;
      p.pose.z=a.z+(b.z-a.z)*t; p.pose.yaw=std::atan2(b.y-a.y,b.x-a.x);
      p.relative_time=time; out.trajectory.points.push_back(p);
      time += config_.spacing/std::max(0.1,in.speed_limit);
    }
    traveled += length * (first_segment ? 1.0 - start_t : 1.0); ++segment;
  }
  // Sparse scenario routes (e.g. mine haul roads) are semantic waypoints,
  // not commands to make a square corner.  Smooth only those short route
  // definitions; ring-road planning has its own lane planner.  The weighted
  // pass preserves the endpoints while rounding each interior corner before
  // obstacle offsets and curvature are computed.
  if (in.reference_path.size() <= 12 && out.trajectory.points.size() > 10) {
    for (int pass = 0; pass < 5; ++pass) {
      auto smoothed = out.trajectory.points;
      for (std::size_t i = 2; i + 2 < out.trajectory.points.size(); ++i) {
        smoothed[i].pose.x = 0.05 * out.trajectory.points[i - 2].pose.x +
                             0.20 * out.trajectory.points[i - 1].pose.x +
                             0.50 * out.trajectory.points[i].pose.x +
                             0.20 * out.trajectory.points[i + 1].pose.x +
                             0.05 * out.trajectory.points[i + 2].pose.x;
        smoothed[i].pose.y = 0.05 * out.trajectory.points[i - 2].pose.y +
                             0.20 * out.trajectory.points[i - 1].pose.y +
                             0.50 * out.trajectory.points[i].pose.y +
                             0.20 * out.trajectory.points[i + 1].pose.y +
                             0.05 * out.trajectory.points[i + 2].pose.y;
        smoothed[i].pose.z = 0.05 * out.trajectory.points[i - 2].pose.z +
                             0.20 * out.trajectory.points[i - 1].pose.z +
                             0.50 * out.trajectory.points[i].pose.z +
                             0.20 * out.trajectory.points[i + 1].pose.z +
                             0.05 * out.trajectory.points[i + 2].pose.z;
      }
      out.trajectory.points.swap(smoothed);
    }
    for (std::size_t i = 1; i < out.trajectory.points.size(); ++i) {
      const auto& a = out.trajectory.points[i - 1].pose;
      auto& p = out.trajectory.points[i];
      p.pose.yaw = std::atan2(p.pose.y - a.y, p.pose.x - a.x);
    }
  }
  // Generic local avoidance: apply one continuous lateral "bump" around
  // each obstacle.  Moving every colliding waypoint independently creates
  // entering/leaving discontinuities (the rectangular loop seen in RViz).
  // The tapered offset keeps heading and curvature continuous and remains
  // completely independent of ring-road geometry.
  for (const auto& obstacle : in.obstacles) {
    const double clearance = obstacle.radius + config_.obstacle_margin;
    std::size_t nearest = out.trajectory.points.size();
    double nearest_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < out.trajectory.points.size(); ++i) {
      const double d = dist(out.trajectory.points[i].pose, obstacle.pose);
      if (d < nearest_distance) {
        nearest_distance = d;
        nearest = i;
      }
    }
    if (nearest == out.trajectory.points.size() || nearest_distance >= clearance + 1.5)
      continue;

    const auto& centre = out.trajectory.points[nearest];
    const double nx = -std::sin(centre.pose.yaw);
    const double ny = std::cos(centre.pose.yaw);
    const double signed_side = (centre.pose.x - obstacle.pose.x) * nx +
                               (centre.pose.y - obstacle.pose.y) * ny;
    // A roadside object may be close in Euclidean distance while still being
    // outside the drivable corridor.  It must not cause a lane change unless
    // its inflated footprint actually intrudes into that corridor.
    if (std::abs(signed_side) > clearance + 0.2)
      continue;
    // If the obstacle is centred on the reference line, keep the benchmark
    // deterministic by selecting the right-hand side.
    const double side = std::abs(signed_side) < 1e-3
                            ? -1.0
                            : (signed_side >= 0.0 ? 1.0 : -1.0);
    const auto window = static_cast<std::size_t>(std::max(
        8.0, std::ceil(3.0 / std::max(0.05, config_.spacing))));
    const double shift = side * (clearance + 0.25);
    const std::size_t begin = nearest > window ? nearest - window : 0;
    const std::size_t end = std::min(out.trajectory.points.size() - 1, nearest + window);
    for (std::size_t i = begin; i <= end; ++i) {
      // Profile around the actual nearest point, not around a clipped
      // begin/end pair.  When the obstacle is near the local horizon edge,
      // using the clipped pair moves the peak and creates a sharp spike.
      const double normalized_distance =
          std::abs(static_cast<double>(i) - static_cast<double>(nearest)) /
          std::max(1.0, static_cast<double>(window));
      const double taper = normalized_distance >= 1.0
                               ? 0.0
                               : 0.5 * (1.0 + std::cos(M_PI * normalized_distance));
      out.trajectory.points[i].pose.x += nx * shift * taper;
      out.trajectory.points[i].pose.y += ny * shift * taper;
    }
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
