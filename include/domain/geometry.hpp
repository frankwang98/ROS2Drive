#ifndef SELF_DRIVING_CAR_DOMAIN_GEOMETRY_HPP
#define SELF_DRIVING_CAR_DOMAIN_GEOMETRY_HPP

namespace sdc {

// Legacy visualization/simulation geometry. Runtime-facing code uses
// domain::Pose2D and domain::Obstacle instead.
struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

struct Obstacle {
  Vec2 position;
  double radius = 1.0;
};

}  // namespace sdc

#endif  // SELF_DRIVING_CAR_DOMAIN_GEOMETRY_HPP
