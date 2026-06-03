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

  [[nodiscard]] int widthCells() const { return width_cells_; }
  [[nodiscard]] int heightCells() const { return height_cells_; }
  [[nodiscard]] double widthMeters() const { return width_m_; }
  [[nodiscard]] double heightMeters() const { return height_m_; }
  [[nodiscard]] double resolution() const { return resolution_m_; }

  [[nodiscard]] bool inBounds(int x, int y) const;
  [[nodiscard]] bool occupied(int x, int y) const;
  void setOccupied(int x, int y, bool occupied);
  [[nodiscard]] Point2D cellCenter(int x, int y) const;
  [[nodiscard]] std::uint32_t occupiedCount() const;

  void saveCsv(const std::filesystem::path& path) const;
  void saveObstaclePoints(const std::filesystem::path& path) const;

 private:
  [[nodiscard]] std::size_t index(int x, int y) const;

  double width_m_ = 0.0;
  double height_m_ = 0.0;
  double resolution_m_ = 0.0;
  int width_cells_ = 0;
  int height_cells_ = 0;
  std::vector<std::uint8_t> cells_;
};

GridMap makeRandomObstacleMap(double width_m,
                              double height_m,
                              double resolution_m,
                              double obstacle_probability,
                              std::uint32_t seed);

std::vector<Pose2D> makeRandomPoses(std::size_t count,
                                    double width_m,
                                    double height_m,
                                    std::uint32_t seed);

}  // namespace collision_benchmark
