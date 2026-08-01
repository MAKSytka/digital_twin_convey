cmake_minimum_required(VERSION 3.8)
project(singulator_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ROLLER_WORLD_GENERATOR
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_matrix_14x4_stream_v2.py
)
set(GENERATED_WORLD_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_worlds)
set(GENERATED_ROLLER_WORLD
  ${GENERATED_WORLD_DIR}/matrix_14x4_stream_v2.sdf
)

add_custom_command(
  OUTPUT ${GENERATED_ROLLER_WORLD}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_WORLD_DIR}
  COMMAND ${Python3_EXECUTABLE} ${ROLLER_WORLD_GENERATOR}
          --output ${GENERATED_ROLLER_WORLD}
  DEPENDS ${ROLLER_WORLD_GENERATOR}
  COMMENT "Generating aligned 18x4 roller-throat world"
  VERBATIM
)

add_custom_target(
  generate_roller_world ALL
  DEPENDS ${GENERATED_ROLLER_WORLD}
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  PATTERN "matrix_14x4_stream_v2.sdf" EXCLUDE
)
install(
  FILES ${GENERATED_ROLLER_WORLD}
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
