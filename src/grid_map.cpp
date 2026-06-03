#include "collision_benchmark/grid_map.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

namespace collision_benchmark {
namespace {

constexpr double kPi = 3.14159265358979323846;

enum class ClusterShape {
  Circle,
  Square,
  Strip,
};

int toCellCount(double meters, double resolution) {
  return static_cast<int>(std::lround(meters / resolution));
}

void addUniqueCell(std::vector<std::pair<int, int>>& cells, int x, int y) {
  const auto cell = std::pair<int, int>{x, y};
  if (std::find(cells.begin(), cells.end(), cell) == cells.end()) {
    cells.push_back(cell);
  }
}

std::vector<std::pair<int, int>> makeCircleClusterCells(int center_x,
                                                        int center_y,
                                                        int requested_count) {
  int radius = std::max(1, static_cast<int>(std::ceil(std::sqrt(requested_count / kPi))));
  std::vector<std::pair<int, int>> cells;
  while (static_cast<int>(cells.size()) < requested_count) {
    cells.clear();
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        if (dx * dx + dy * dy <= radius * radius) {
          cells.emplace_back(center_x + dx, center_y + dy);
        }
      }
    }
    ++radius;
  }

  std::sort(cells.begin(), cells.end(), [center_x, center_y](const auto& lhs, const auto& rhs) {
    const int lhs_dx = lhs.first - center_x;
    const int lhs_dy = lhs.second - center_y;
    const int rhs_dx = rhs.first - center_x;
    const int rhs_dy = rhs.second - center_y;
    return lhs_dx * lhs_dx + lhs_dy * lhs_dy < rhs_dx * rhs_dx + rhs_dy * rhs_dy;
  });
  cells.resize(static_cast<std::size_t>(requested_count));
  return cells;
}

std::vector<std::pair<int, int>> makeSquareClusterCells(int center_x,
                                                        int center_y,
                                                        int requested_count) {
  const int side = std::max(1, static_cast<int>(std::ceil(std::sqrt(requested_count))));
  const int half = side / 2;
  std::vector<std::pair<int, int>> cells;
  cells.reserve(static_cast<std::size_t>(side * side));
  for (int dy = -half; dy < side - half; ++dy) {
    for (int dx = -half; dx < side - half; ++dx) {
      cells.emplace_back(center_x + dx, center_y + dy);
    }
  }

  std::sort(cells.begin(), cells.end(), [center_x, center_y](const auto& lhs, const auto& rhs) {
    const int lhs_chebyshev =
        std::max(std::abs(lhs.first - center_x), std::abs(lhs.second - center_y));
    const int rhs_chebyshev =
        std::max(std::abs(rhs.first - center_x), std::abs(rhs.second - center_y));
    return lhs_chebyshev < rhs_chebyshev;
  });
  cells.resize(static_cast<std::size_t>(requested_count));
  return cells;
}

std::vector<std::pair<int, int>> makeStripClusterCells(int center_x,
                                                       int center_y,
                                                       int requested_count,
                                                       std::mt19937& rng) {
  std::uniform_int_distribution<int> width_dist(1, 2);
  std::uniform_real_distribution<double> angle_dist(0.0, kPi);
  const int strip_width = width_dist(rng);
  const int strip_length = std::max(1, (requested_count + strip_width - 1) / strip_width);
  const double angle = angle_dist(rng);
  const double cos_angle = std::cos(angle);
  const double sin_angle = std::sin(angle);
  const double half_length = (static_cast<double>(strip_length) - 1.0) * 0.5;
  const double half_width = (static_cast<double>(strip_width) - 1.0) * 0.5;

  std::vector<std::pair<int, int>> cells;
  cells.reserve(static_cast<std::size_t>(requested_count));
  for (int i = 0; i < strip_length && static_cast<int>(cells.size()) < requested_count; ++i) {
    for (int j = 0; j < strip_width && static_cast<int>(cells.size()) < requested_count; ++j) {
      const double local_x = static_cast<double>(i) - half_length;
      const double local_y = static_cast<double>(j) - half_width;
      const int x = center_x + static_cast<int>(std::lround(local_x * cos_angle - local_y * sin_angle));
      const int y = center_y + static_cast<int>(std::lround(local_x * sin_angle + local_y * cos_angle));
      addUniqueCell(cells, x, y);
    }
  }

  int extension = 1;
  while (static_cast<int>(cells.size()) < requested_count) {
    const double local_x = half_length + static_cast<double>(extension);
    const int x = center_x + static_cast<int>(std::lround(local_x * cos_angle));
    const int y = center_y + static_cast<int>(std::lround(local_x * sin_angle));
    addUniqueCell(cells, x, y);
    ++extension;
  }
  return cells;
}

std::uint32_t addCluster(GridMap& map,
                         std::mt19937& rng,
                         int requested_count,
                         ClusterShape shape) {
  std::uniform_int_distribution<int> x_dist(0, map.widthCells() - 1);
  std::uniform_int_distribution<int> y_dist(0, map.heightCells() - 1);
  const int center_x = x_dist(rng);
  const int center_y = y_dist(rng);

  std::vector<std::pair<int, int>> cells;
  switch (shape) {
    case ClusterShape::Circle:
      cells = makeCircleClusterCells(center_x, center_y, requested_count);
      break;
    case ClusterShape::Square:
      cells = makeSquareClusterCells(center_x, center_y, requested_count);
      break;
    case ClusterShape::Strip:
      cells = makeStripClusterCells(center_x, center_y, requested_count, rng);
      break;
  }

  std::uint32_t added = 0;
  for (const auto& [x, y] : cells) {
    if (!map.inBounds(x, y) || map.occupied(x, y)) {
      continue;
    }
    map.setOccupied(x, y, true);
    ++added;
  }
  return added;
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
  occupancy_prefix_valid_ = false;
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

std::uint32_t GridMap::occupiedCountInRange(int min_x,
                                            int min_y,
                                            int max_x,
                                            int max_y) const {
  if (max_x < min_x || max_y < min_y) {
    return 0;
  }
  min_x = std::clamp(min_x, 0, width_cells_ - 1);
  min_y = std::clamp(min_y, 0, height_cells_ - 1);
  max_x = std::clamp(max_x, 0, width_cells_ - 1);
  max_y = std::clamp(max_y, 0, height_cells_ - 1);
  ensureOccupancyPrefix();

  const int x0 = min_x;
  const int y0 = min_y;
  const int x1 = max_x + 1;
  const int y1 = max_y + 1;
  return occupancy_prefix_[prefixIndex(x1, y1)] -
         occupancy_prefix_[prefixIndex(x0, y1)] -
         occupancy_prefix_[prefixIndex(x1, y0)] +
         occupancy_prefix_[prefixIndex(x0, y0)];
}

bool GridMap::hasOccupiedInRange(int min_x, int min_y, int max_x, int max_y) const {
  return occupiedCountInRange(min_x, min_y, max_x, max_y) > 0;
}

void GridMap::prepareRangeQueries() const {
  ensureOccupancyPrefix();
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

std::size_t GridMap::prefixIndex(int x, int y) const {
  return static_cast<std::size_t>(y * (width_cells_ + 1) + x);
}

void GridMap::ensureOccupancyPrefix() const {
  if (occupancy_prefix_valid_) {
    return;
  }

  occupancy_prefix_.assign(
      static_cast<std::size_t>((width_cells_ + 1) * (height_cells_ + 1)), 0);
  for (int y = 0; y < height_cells_; ++y) {
    std::uint32_t row_sum = 0;
    for (int x = 0; x < width_cells_; ++x) {
      row_sum += cells_[index(x, y)] != 0 ? 1U : 0U;
      occupancy_prefix_[prefixIndex(x + 1, y + 1)] =
          occupancy_prefix_[prefixIndex(x + 1, y)] + row_sum;
    }
  }
  occupancy_prefix_valid_ = true;
}

GridMap makeRandomObstacleMap(double width_m,
                              double height_m,
                              double resolution_m,
                              double obstacle_density,
                              std::uint32_t seed) {
  if (obstacle_density < 0.0 || obstacle_density > 1.0) {
    throw std::invalid_argument("obstacle density must be in [0, 1]");
  }

  GridMap map(width_m, height_m, resolution_m);
  std::mt19937 rng(seed);
  const auto total_cells =
      static_cast<std::uint32_t>(map.widthCells() * map.heightCells());
  const auto target_obstacle_count =
      static_cast<std::uint32_t>(std::lround(total_cells * obstacle_density));
  std::uniform_int_distribution<int> cluster_size_dist(5, 50);
  std::uniform_int_distribution<int> shape_dist(0, 2);

  std::uint32_t occupied_count = 0;
  std::uint32_t attempts = 0;
  const std::uint32_t max_attempts = target_obstacle_count * 4U + 100U;
  while (occupied_count < target_obstacle_count && attempts < max_attempts) {
    const auto remaining = static_cast<int>(target_obstacle_count - occupied_count);
    const int requested_count = std::min(cluster_size_dist(rng), remaining);
    const auto shape = static_cast<ClusterShape>(shape_dist(rng));
    occupied_count += addCluster(map, rng, requested_count, shape);
    ++attempts;
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
