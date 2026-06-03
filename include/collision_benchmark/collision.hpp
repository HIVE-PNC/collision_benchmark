#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "collision_benchmark/grid_map.hpp"
#include "collision_benchmark/types.hpp"

namespace collision_benchmark {

enum class CollisionAlgorithm {
  AabbOnly,
  AabbObb,
  AabbTriangleArea,
  FixedAabbOnly,
  FixedAabbObb,
  FixedAabbTriangleArea,
  Circle,
  TwoCircles,
};

struct BenchmarkResult {
  std::string name;
  std::size_t pose_count = 0;
  std::size_t collision_count = 0;
  double elapsed_ms = 0.0;
  double average_us = 0.0;
};

RectangleCorners vehicleCorners(const Pose2D& pose,
                                              const VehicleShape& shape);
AABB boundingBox(const RectangleCorners& corners);

bool collidesAabbOnly(const GridMap& map,
                                    const Pose2D& pose,
                                    const VehicleShape& shape);
bool collidesAabbObb(const GridMap& map,
                                   const Pose2D& pose,
                                   const VehicleShape& shape);
bool collidesAabbTriangleArea(const GridMap& map,
                                            const Pose2D& pose,
                                            const VehicleShape& shape);
bool collidesFixedAabbOnly(const GridMap& map,
                                         const Pose2D& pose,
                                         const VehicleShape& shape);
bool collidesFixedAabbObb(const GridMap& map,
                                        const Pose2D& pose,
                                        const VehicleShape& shape);
bool collidesFixedAabbTriangleArea(const GridMap& map,
                                                 const Pose2D& pose,
                                                 const VehicleShape& shape);
bool collidesCircle(const GridMap& map,
                                  const Pose2D& pose,
                                  const VehicleShape& shape);
bool collidesTwoCircles(const GridMap& map,
                                      const Pose2D& pose,
                                      const VehicleShape& shape);

BenchmarkResult runBenchmark(CollisionAlgorithm algorithm,
                             const GridMap& map,
                             const std::vector<Pose2D>& poses,
                             const VehicleShape& shape);

}  // namespace collision_benchmark
