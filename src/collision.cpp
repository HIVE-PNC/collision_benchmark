#include "collision_benchmark/collision.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

namespace collision_benchmark {
namespace {

struct CellRange {
  int min_x = 0;
  int min_y = 0;
  int max_x = -1;
  int max_y = -1;
};

CellRange aabbToCellRange(const GridMap& map, const AABB& aabb) {
  const double inv_resolution = 1.0 / map.resolution();
  CellRange range;
  range.min_x = static_cast<int>(std::floor(aabb.min_x * inv_resolution));
  range.min_y = static_cast<int>(std::floor(aabb.min_y * inv_resolution));
  range.max_x = static_cast<int>(std::floor(aabb.max_x * inv_resolution));
  range.max_y = static_cast<int>(std::floor(aabb.max_y * inv_resolution));

  range.min_x = std::clamp(range.min_x, 0, map.widthCells() - 1);
  range.min_y = std::clamp(range.min_y, 0, map.heightCells() - 1);
  range.max_x = std::clamp(range.max_x, 0, map.widthCells() - 1);
  range.max_y = std::clamp(range.max_y, 0, map.heightCells() - 1);
  if (aabb.max_x < 0.0 || aabb.max_y < 0.0 || aabb.min_x > map.widthMeters() ||
      aabb.min_y > map.heightMeters()) {
    return CellRange{};
  }
  return range;
}

AABB fixedMaxOffsetAabb(const Pose2D& pose, const VehicleShape& shape) {
  const double half_length = shape.length * 0.5;
  const double half_width = shape.width * 0.5;
  const double max_offset = std::hypot(half_length, half_width);
  return AABB{pose.x - max_offset,
              pose.y - max_offset,
              pose.x + max_offset,
              pose.y + max_offset};
}

bool pointInObb(const Point2D& point,
                const Pose2D& pose,
                double cos_yaw,
                double sin_yaw,
                const VehicleShape& shape) {
  const double dx = point.x - pose.x;
  const double dy = point.y - pose.y;

  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  return std::abs(local_x) <= shape.length * 0.5 &&
         std::abs(local_y) <= shape.width * 0.5;
}

double triangleArea2(const Point2D& a, const Point2D& b, const Point2D& c) {
  return std::abs((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y));
}

bool pointInRectangleByTriangleArea(const Point2D& point,
                                    const RectangleCorners& corners,
                                    const VehicleShape& shape) {
  const double rectangle_area2 = 2.0 * shape.length * shape.width;
  double area_sum2 = 0.0;
  for (std::size_t i = 0; i < corners.size(); ++i) {
    const auto& a = corners[i];
    const auto& b = corners[(i + 1) % corners.size()];
    area_sum2 += triangleArea2(point, a, b);
  }
  return area_sum2 <= rectangle_area2 + 1e-9;
}

const char* algorithmName(CollisionAlgorithm algorithm) {
  switch (algorithm) {
    case CollisionAlgorithm::AabbOnly:
      return "AABB";
    case CollisionAlgorithm::AabbObb:
      return "AABB + OBB";
    case CollisionAlgorithm::AabbTriangleArea:
      return "AABB + triangle-area";
    case CollisionAlgorithm::FixedAabbObb:
      return "fixed AABB + OBB";
    case CollisionAlgorithm::FixedAabbTriangleArea:
      return "fixed AABB + triangle-area";
  }
  return "unknown";
}

bool runCollision(CollisionAlgorithm algorithm,
                  const GridMap& map,
                  const Pose2D& pose,
                  const VehicleShape& shape) {
  switch (algorithm) {
    case CollisionAlgorithm::AabbOnly:
      return collidesAabbOnly(map, pose, shape);
    case CollisionAlgorithm::AabbObb:
      return collidesAabbObb(map, pose, shape);
    case CollisionAlgorithm::AabbTriangleArea:
      return collidesAabbTriangleArea(map, pose, shape);
    case CollisionAlgorithm::FixedAabbObb:
      return collidesFixedAabbObb(map, pose, shape);
    case CollisionAlgorithm::FixedAabbTriangleArea:
      return collidesFixedAabbTriangleArea(map, pose, shape);
  }
  return false;
}

}  // namespace

RectangleCorners vehicleCorners(const Pose2D& pose, const VehicleShape& shape) {
  const double half_length = shape.length * 0.5;
  const double half_width = shape.width * 0.5;
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  const std::array<Point2D, 4> local = {
      Point2D{half_length, half_width},
      Point2D{half_length, -half_width},
      Point2D{-half_length, -half_width},
      Point2D{-half_length, half_width},
  };

  RectangleCorners corners{};
  for (std::size_t i = 0; i < local.size(); ++i) {
    corners[i] = Point2D{pose.x + local[i].x * cos_yaw - local[i].y * sin_yaw,
                         pose.y + local[i].x * sin_yaw + local[i].y * cos_yaw};
  }
  return corners;
}

AABB boundingBox(const RectangleCorners& corners) {
  AABB aabb;
  aabb.min_x = std::numeric_limits<double>::infinity();
  aabb.min_y = std::numeric_limits<double>::infinity();
  aabb.max_x = -std::numeric_limits<double>::infinity();
  aabb.max_y = -std::numeric_limits<double>::infinity();
  for (const auto& point : corners) {
    aabb.min_x = std::min(aabb.min_x, point.x);
    aabb.min_y = std::min(aabb.min_y, point.y);
    aabb.max_x = std::max(aabb.max_x, point.x);
    aabb.max_y = std::max(aabb.max_y, point.y);
  }
  return aabb;
}

bool collidesAabbOnly(const GridMap& map, const Pose2D& pose, const VehicleShape& shape) {
  const auto range = aabbToCellRange(map, boundingBox(vehicleCorners(pose, shape)));
  for (int y = range.min_y; y <= range.max_y; ++y) {
    for (int x = range.min_x; x <= range.max_x; ++x) {
      if (map.occupied(x, y)) {
        return true;
      }
    }
  }
  return false;
}

bool collidesAabbObb(const GridMap& map, const Pose2D& pose, const VehicleShape& shape) {
  const auto range = aabbToCellRange(map, boundingBox(vehicleCorners(pose, shape)));
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  for (int y = range.min_y; y <= range.max_y; ++y) {
    for (int x = range.min_x; x <= range.max_x; ++x) {
      if (!map.occupied(x, y)) {
        continue;
      }
      if (pointInObb(map.cellCenter(x, y), pose, cos_yaw, sin_yaw, shape)) {
        return true;
      }
    }
  }
  return false;
}

bool collidesAabbTriangleArea(const GridMap& map,
                              const Pose2D& pose,
                              const VehicleShape& shape) {
  const auto corners = vehicleCorners(pose, shape);
  const auto range = aabbToCellRange(map, boundingBox(corners));
  for (int y = range.min_y; y <= range.max_y; ++y) {
    for (int x = range.min_x; x <= range.max_x; ++x) {
      if (!map.occupied(x, y)) {
        continue;
      }
      if (pointInRectangleByTriangleArea(map.cellCenter(x, y), corners, shape)) {
        return true;
      }
    }
  }
  return false;
}

bool collidesFixedAabbObb(const GridMap& map, const Pose2D& pose, const VehicleShape& shape) {
  const auto range = aabbToCellRange(map, fixedMaxOffsetAabb(pose, shape));
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  for (int y = range.min_y; y <= range.max_y; ++y) {
    for (int x = range.min_x; x <= range.max_x; ++x) {
      if (!map.occupied(x, y)) {
        continue;
      }
      if (pointInObb(map.cellCenter(x, y), pose, cos_yaw, sin_yaw, shape)) {
        return true;
      }
    }
  }
  return false;
}

bool collidesFixedAabbTriangleArea(const GridMap& map,
                                   const Pose2D& pose,
                                   const VehicleShape& shape) {
  const auto range = aabbToCellRange(map, fixedMaxOffsetAabb(pose, shape));
  const auto corners = vehicleCorners(pose, shape);
  for (int y = range.min_y; y <= range.max_y; ++y) {
    for (int x = range.min_x; x <= range.max_x; ++x) {
      if (!map.occupied(x, y)) {
        continue;
      }
      if (pointInRectangleByTriangleArea(map.cellCenter(x, y), corners, shape)) {
        return true;
      }
    }
  }
  return false;
}

BenchmarkResult runBenchmark(CollisionAlgorithm algorithm,
                             const GridMap& map,
                             const std::vector<Pose2D>& poses,
                             const VehicleShape& shape) {
  std::size_t collisions = 0;
  volatile std::size_t sink = 0;

  const auto begin = std::chrono::steady_clock::now();
  for (const auto& pose : poses) {
    const bool collided = runCollision(algorithm, map, pose, shape);
    collisions += collided ? 1U : 0U;
    sink += collided ? 1U : 0U;
  }
  const auto end = std::chrono::steady_clock::now();
  (void)sink;

  const auto elapsed_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  const double elapsed_ms = static_cast<double>(elapsed_ns) / 1'000'000.0;
  const double average_us =
      poses.empty() ? 0.0 : static_cast<double>(elapsed_ns) / 1'000.0 / poses.size();

  return BenchmarkResult{algorithmName(algorithm),
                         poses.size(),
                         collisions,
                         elapsed_ms,
                         average_us};
}

}  // namespace collision_benchmark
