#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "collision_benchmark/types.hpp"

namespace collision_benchmark {

class GridMap {
 public:
  GridMap(double width_m, double height_m, double resolution_m);

  int widthCells() const { return width_cells_; }
  int heightCells() const { return height_cells_; }
  double widthMeters() const { return width_m_; }
  double heightMeters() const { return height_m_; }
  double resolution() const { return resolution_m_; }

  bool inBounds(int x, int y) const;
  bool occupied(int x, int y) const;
  void setOccupied(int x, int y, bool occupied);
  Point2D cellCenter(int x, int y) const;
  std::uint32_t occupiedCount() const;
  std::uint32_t occupiedCountInRange(int min_x,
                                                   int min_y,
                                                   int max_x,
                                                   int max_y) const;
  bool hasOccupiedInRange(int min_x, int min_y, int max_x, int max_y) const;
  void prepareRangeQueries() const;

  void saveCsv(const std::filesystem::path& path) const;
  void saveObstaclePoints(const std::filesystem::path& path) const;

 private:
  std::size_t index(int x, int y) const;
  std::size_t prefixIndex(int x, int y) const;
  void ensureOccupancyPrefix() const;

  double width_m_ = 0.0;
  double height_m_ = 0.0;
  double resolution_m_ = 0.0;
  int width_cells_ = 0;
  int height_cells_ = 0;
  std::vector<std::uint8_t> cells_;
  mutable bool occupancy_prefix_valid_ = false;
  mutable std::vector<std::uint32_t> occupancy_prefix_;
};

GridMap makeRandomObstacleMap(double width_m,
                              double height_m,
                              double resolution_m,
                              double obstacle_density,
                              std::uint32_t seed);

std::vector<Pose2D> makeRandomPoses(std::size_t count,
                                    double width_m,
                                    double height_m,
                                    std::uint32_t seed);

}  // namespace collision_benchmark
