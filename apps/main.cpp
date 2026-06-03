#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

#include "collision_benchmark/collision.hpp"
#include "collision_benchmark/grid_map.hpp"

namespace {

constexpr double kMapWidthM = 100.0;
constexpr double kMapHeightM = 100.0;
constexpr double kResolutionM = 0.25;
constexpr double kObstacleDensity = 0.02;
constexpr std::uint32_t kMapSeed = 20260602;
constexpr std::uint32_t kPoseSeed = 20260603;
constexpr std::size_t kPoseCount = 100000;

double falsePositiveRate(const collision_benchmark::BenchmarkResult& result,
                         std::size_t reference_collision_count) {
  if (result.pose_count == 0 || result.collision_count <= reference_collision_count) {
    return 0.0;
  }
  const auto false_positive_count = result.collision_count - reference_collision_count;
  return static_cast<double>(false_positive_count) * 100.0 /
         static_cast<double>(result.pose_count);
}

void printResult(const collision_benchmark::BenchmarkResult& result,
                 std::size_t reference_collision_count) {
  std::cout << std::left << std::setw(30) << result.name
            << std::right << std::setw(12) << result.pose_count
            << std::setw(14) << result.collision_count
            << std::setw(16) << std::fixed << std::setprecision(3) << result.elapsed_ms
            << std::setw(16) << std::fixed << std::setprecision(3) << result.average_us
            << std::setw(16) << std::fixed << std::setprecision(3)
            << falsePositiveRate(result, reference_collision_count)
            << '\n';
}

}  // namespace

int main() {
  try {
    const auto output_dir = std::filesystem::path("data");
    auto map = collision_benchmark::makeRandomObstacleMap(
        kMapWidthM, kMapHeightM, kResolutionM, kObstacleDensity, kMapSeed);
    map.prepareRangeQueries();
    map.saveCsv(output_dir / "grid_map.csv");
    map.saveObstaclePoints(output_dir / "obstacles_xy.csv");

    const auto poses =
        collision_benchmark::makeRandomPoses(kPoseCount, kMapWidthM, kMapHeightM, kPoseSeed);
    const collision_benchmark::VehicleShape vehicle{4.0, 2.0};

    std::cout << "Map: " << kMapWidthM << "m x " << kMapHeightM << "m, resolution "
              << kResolutionM << "m, cells " << map.widthCells() << " x "
              << map.heightCells() << ", occupied " << map.occupiedCount()
              << ", density " << kObstacleDensity << ", map_seed " << kMapSeed
              << ", pose_seed " << kPoseSeed << '\n';
    std::cout << "Vehicle: length " << vehicle.length << "m, width " << vehicle.width
              << "m, poses " << poses.size() << "\n\n";

    const std::vector<collision_benchmark::CollisionAlgorithm> algorithms = {
        collision_benchmark::CollisionAlgorithm::AabbOnly,
        collision_benchmark::CollisionAlgorithm::AabbObb,
        collision_benchmark::CollisionAlgorithm::AabbTriangleArea,
        collision_benchmark::CollisionAlgorithm::FixedAabbOnly,
        collision_benchmark::CollisionAlgorithm::FixedAabbObb,
        collision_benchmark::CollisionAlgorithm::FixedAabbTriangleArea,
        collision_benchmark::CollisionAlgorithm::Circle,
        collision_benchmark::CollisionAlgorithm::TwoCircles,
    };

    std::vector<collision_benchmark::BenchmarkResult> results;
    results.reserve(algorithms.size());
    std::size_t reference_collision_count = 0;
    for (const auto algorithm : algorithms) {
      auto result = collision_benchmark::runBenchmark(algorithm, map, poses, vehicle);
      if (algorithm == collision_benchmark::CollisionAlgorithm::AabbObb) {
        reference_collision_count = result.collision_count;
      }
      results.push_back(std::move(result));
    }

    std::cout << "Reference: AABB + OBB collision count = "
              << reference_collision_count << "\n\n";

    std::cout << std::left << std::setw(30) << "algorithm"
              << std::right << std::setw(12) << "poses"
              << std::setw(14) << "collisions"
              << std::setw(16) << "total_ms"
              << std::setw(16) << "avg_us"
              << std::setw(16) << "false_pos_%" << '\n';
    std::cout << std::string(104, '-') << '\n';

    for (const auto& result : results) {
      printResult(result, reference_collision_count);
    }

    std::cout << "\nSaved map files:\n";
    std::cout << "  " << (output_dir / "grid_map.csv") << '\n';
    std::cout << "  " << (output_dir / "obstacles_xy.csv") << '\n';
  } catch (const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
