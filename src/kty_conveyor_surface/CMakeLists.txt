cmake_minimum_required(VERSION 3.16)
project(kty_conveyor_surface)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(KtyConveyorSurfaceSystem SHARED
  src/KtyConveyorSurfaceSystem.cc
)

target_compile_features(KtyConveyorSurfaceSystem PUBLIC cxx_std_17)
target_compile_options(KtyConveyorSurfaceSystem PRIVATE -Wall -Wextra -Wpedantic)

target_link_libraries(KtyConveyorSurfaceSystem
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(
  TARGETS KtyConveyorSurfaceSystem
  LIBRARY DESTINATION lib
)

ament_package()
