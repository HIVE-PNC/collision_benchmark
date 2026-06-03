#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "collision_benchmark/collision.hpp"
#include "collision_benchmark/grid_map.hpp"

namespace {

constexpr double kMapWidthM = 100.0;
constexpr double kMapHeightM = 100.0;
constexpr double kResolutionM = 0.25;
constexpr double kObstacleProbability = 0.05;
constexpr std::uint32_t kMapSeed = 20260602;
constexpr std::uint32_t kPoseSeed = 20260603;
constexpr std::size_t kPoseCount = 100000;

void printResult(const collision_benchmark::BenchmarkResult& result) {
  std::cout << std::left << std::setw(30) << result.name
            << std::right << std::setw(12) << result.pose_count
            << std::setw(14) << result.collision_count
            << std::setw(16) << std::fixed << std::setprecision(3) << result.elapsed_ms
            << std::setw(16) << std::fixed << std::setprecision(3) << result.average_us
            << '\n';
}

}  // namespace

int main() {
  try {
    const auto output_dir = std::filesystem::path("data");
    auto map = collision_benchmark::makeRandomObstacleMap(
        kMapWidthM, kMapHeightM, kResolutionM, kObstacleProbability, kMapSeed);
    map.saveCsv(output_dir / "grid_map.csv");
    map.saveObstaclePoints(output_dir / "obstacles_xy.csv");

    const auto poses =
        collision_benchmark::makeRandomPoses(kPoseCount, kMapWidthM, kMapHeightM, kPoseSeed);
    const collision_benchmark::VehicleShape vehicle{4.0, 2.0};

    std::cout << "Map: " << kMapWidthM << "m x " << kMapHeightM << "m, resolution "
              << kResolutionM << "m, cells " << map.widthCells() << " x "
              << map.heightCells() << ", occupied " << map.occupiedCount()
              << ", map_seed " << kMapSeed << ", pose_seed " << kPoseSeed << '\n';
    std::cout << "Vehicle: length " << vehicle.length << "m, width " << vehicle.width
              << "m, poses " << poses.size() << "\n\n";

    std::cout << std::left << std::setw(30) << "algorithm"
              << std::right << std::setw(12) << "poses"
              << std::setw(14) << "collisions"
              << std::setw(16) << "total_ms"
              << std::setw(16) << "avg_us" << '\n';
    std::cout << std::string(88, '-') << '\n';

    const std::vector<collision_benchmark::CollisionAlgorithm> algorithms = {
        collision_benchmark::CollisionAlgorithm::AabbOnly,
        collision_benchmark::CollisionAlgorithm::AabbObb,
        collision_benchmark::CollisionAlgorithm::AabbTriangleArea,
        collision_benchmark::CollisionAlgorithm::FixedAabbObb,
        collision_benchmark::CollisionAlgorithm::FixedAabbTriangleArea,
        collision_benchmark::CollisionAlgorithm::Circle,
        collision_benchmark::CollisionAlgorithm::TwoCircles,
    };

    for (const auto algorithm : algorithms) {
      printResult(collision_benchmark::runBenchmark(algorithm, map, poses, vehicle));
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
