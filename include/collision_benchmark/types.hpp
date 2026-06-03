#pragma once

#include <array>
#include <cstdint>

namespace collision_benchmark {

struct Point2D {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct AABB {
  double min_x = 0.0;
  double min_y = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
};

struct VehicleShape {
  double length = 4.0;
  double width = 2.0;
};

using RectangleCorners = std::array<Point2D, 4>;

}  // namespace collision_benchmark
