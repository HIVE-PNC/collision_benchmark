cmake_minimum_required(VERSION 3.16)

project(collision_benchmark
  VERSION 0.1.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(collision_benchmark_lib
  src/collision.cpp
  src/grid_map.cpp
)

target_include_directories(collision_benchmark_lib
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

target_compile_options(collision_benchmark_lib
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)

add_executable(collision_benchmark
  apps/main.cpp
)

target_link_libraries(collision_benchmark
  PRIVATE
    collision_benchmark_lib
)

target_compile_options(collision_benchmark
  PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
)
