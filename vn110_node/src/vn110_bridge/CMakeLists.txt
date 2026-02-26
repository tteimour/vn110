cmake_minimum_required(VERSION 3.8)
project(vn110_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

set(VNSDK_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../vnsdk" CACHE PATH "Path to VectorNav SDK")
message(STATUS "Using VectorNav SDK at: ${VNSDK_PATH}")

file(GLOB_RECURSE VNSDK_SOURCES
    "${VNSDK_PATH}/src/HAL/*.cpp"
    "${VNSDK_PATH}/src/Implementation/*.cpp"
    "${VNSDK_PATH}/src/Interface/*.cpp"
)

add_library(vnsdk STATIC ${VNSDK_SOURCES})
target_include_directories(vnsdk PUBLIC ${VNSDK_PATH}/include)
target_compile_definitions(vnsdk PUBLIC GNSS_GROUP_ENABLE=0xFFFFFFFF)
target_compile_definitions(vnsdk PUBLIC GNSS2_GROUP_ENABLE=0xFFFFFFFF)
set_target_properties(vnsdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

include_directories(include)

add_executable(vn110_bridge_node src/vn110_bridge.cpp src/dead_reckoning.cpp)
target_include_directories(vn110_bridge_node PRIVATE ${VNSDK_PATH}/include)
target_link_libraries(vn110_bridge_node vnsdk pthread)
ament_target_dependencies(vn110_bridge_node
  rclcpp
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  px4_msgs
  ament_index_cpp
)

install(TARGETS vn110_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
