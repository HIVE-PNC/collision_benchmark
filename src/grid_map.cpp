#include "collision_benchmark/grid_map.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>

namespace collision_benchmark {
namespace {

constexpr double kPi = 3.14159265358979323846;

int toCellCount(double meters, double resolution) {
  return static_cast<int>(std::lround(meters / resolution));
}

}  // namespace

GridMap::GridMap(double width_m, double height_m, double resolution_m)
    : width_m_(width_m),
      height_m_(height_m),
      resolution_m_(resolution_m),
      width_cells_(toCellCount(width_m, resolution_m)),
      height_cells_(toCellCount(height_m, resolution_m)),
      cells_(static_cast<std::size_t>(width_cells_ * height_cells_), 0) {
  if (width_m <= 0.0 || height_m <= 0.0 || resolution_m <= 0.0) {
    throw std::invalid_argument("map dimensions and resolution must be positive");
  }
}

bool GridMap::inBounds(int x, int y) const {
  return x >= 0 && y >= 0 && x < width_cells_ && y < height_cells_;
}

bool GridMap::occupied(int x, int y) const {
  return inBounds(x, y) && cells_[index(x, y)] != 0;
}

void GridMap::setOccupied(int x, int y, bool occupied_cell) {
  if (!inBounds(x, y)) {
    throw std::out_of_range("cell index out of range");
  }
  cells_[index(x, y)] = occupied_cell ? 1U : 0U;
}

Point2D GridMap::cellCenter(int x, int y) const {
  return Point2D{(static_cast<double>(x) + 0.5) * resolution_m_,
                 (static_cast<double>(y) + 0.5) * resolution_m_};
}

std::uint32_t GridMap::occupiedCount() const {
  std::uint32_t count = 0;
  for (const auto cell : cells_) {
    count += cell != 0 ? 1U : 0U;
  }
  return count;
}

void GridMap::saveCsv(const std::filesystem::path& path) const {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to open map csv for writing: " + path.string());
  }

  out << "width_m," << width_m_ << "\n";
  out << "height_m," << height_m_ << "\n";
  out << "resolution_m," << resolution_m_ << "\n";
  out << "width_cells," << width_cells_ << "\n";
  out << "height_cells," << height_cells_ << "\n";
  out << "grid\n";
  for (int y = height_cells_ - 1; y >= 0; --y) {
    for (int x = 0; x < width_cells_; ++x) {
      out << static_cast<int>(cells_[index(x, y)]);
      if (x + 1 < width_cells_) {
        out << ',';
      }
    }
    out << '\n';
  }
}

void GridMap::saveObstaclePoints(const std::filesystem::path& path) const {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("failed to open obstacle points for writing: " + path.string());
  }

  out << std::fixed << std::setprecision(3);
  out << "x_m,y_m\n";
  for (int y = 0; y < height_cells_; ++y) {
    for (int x = 0; x < width_cells_; ++x) {
      if (!occupied(x, y)) {
        continue;
      }
      const auto center = cellCenter(x, y);
      out << center.x << ',' << center.y << '\n';
    }
  }
}

std::size_t GridMap::index(int x, int y) const {
  return static_cast<std::size_t>(y * width_cells_ + x);
}

GridMap makeRandomObstacleMap(double width_m,
                              double height_m,
                              double resolution_m,
                              double obstacle_probability,
                              std::uint32_t seed) {
  if (obstacle_probability < 0.0 || obstacle_probability > 1.0) {
    throw std::invalid_argument("obstacle probability must be in [0, 1]");
  }

  GridMap map(width_m, height_m, resolution_m);
  std::mt19937 rng(seed);
  std::bernoulli_distribution occupied(obstacle_probability);
  for (int y = 0; y < map.heightCells(); ++y) {
    for (int x = 0; x < map.widthCells(); ++x) {
      map.setOccupied(x, y, occupied(rng));
    }
  }
  return map;
}

std::vector<Pose2D> makeRandomPoses(std::size_t count,
                                    double width_m,
                                    double height_m,
                                    std::uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> x_dist(0.0, width_m);
  std::uniform_real_distribution<double> y_dist(0.0, height_m);
  std::uniform_real_distribution<double> yaw_dist(-kPi, kPi);

  std::vector<Pose2D> poses;
  poses.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    poses.push_back(Pose2D{x_dist(rng), y_dist(rng), yaw_dist(rng)});
  }
  return poses;
}

}  // namespace collision_benchmark
