cmake_minimum_required(VERSION 3.8)
project(aero_halo_360 VERSION 0.2.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SectorDistances.msg"
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME}_core
  src/sectorizer.cpp
  src/temporal_filter.cpp
  src/inflation.cpp
  src/self_mask.cpp
  src/source_timestamp.cpp
  src/pointcloud_validation.cpp
  src/watchdog.cpp
  src/diagnostics.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  diagnostic_msgs
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

add_executable(cloud_processor_node src/cloud_processor_node.cpp)
target_include_directories(cloud_processor_node PRIVATE include)
target_link_libraries(cloud_processor_node ${PROJECT_NAME}_core)
ament_target_dependencies(cloud_processor_node
  diagnostic_msgs
  geometry_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cloud_processor_node "${cpp_typesupport_target}")

install(TARGETS
  ${PROJECT_NAME}_core
  cloud_processor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/mavlink_obstacle_sender.py
  scripts/check_mavlink.py
  scripts/check_sector_directions.py
  scripts/benchmark_latency.py
  scripts/replay_to_mavlink.py
  scripts/synthetic_cloud_generator.py
  scripts/plot_sector_log.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_add_gtest(test_sector_mapping test/test_sector_mapping.cpp)
  target_link_libraries(test_sector_mapping ${PROJECT_NAME}_core)
  target_include_directories(test_sector_mapping PRIVATE include)

  ament_add_gtest(test_temporal_filter test/test_temporal_filter.cpp)
  target_link_libraries(test_temporal_filter ${PROJECT_NAME}_core)
  target_include_directories(test_temporal_filter PRIVATE include)

  ament_add_gtest(test_inflation test/test_inflation.cpp)
  target_link_libraries(test_inflation ${PROJECT_NAME}_core)
  target_include_directories(test_inflation PRIVATE include)

  ament_add_gtest(test_self_mask test/test_self_mask.cpp)
  target_link_libraries(test_self_mask ${PROJECT_NAME}_core)
  target_include_directories(test_self_mask PRIVATE include)

  ament_add_gtest(test_watchdog test/test_watchdog.cpp)
  target_link_libraries(test_watchdog ${PROJECT_NAME}_core)
  target_include_directories(test_watchdog PRIVATE include)

  ament_add_gtest(test_source_timestamp test/test_source_timestamp.cpp)
  target_link_libraries(test_source_timestamp ${PROJECT_NAME}_core)
  target_include_directories(test_source_timestamp PRIVATE include)

  ament_add_gtest(test_pointcloud_validation test/test_pointcloud_validation.cpp)
  target_link_libraries(test_pointcloud_validation ${PROJECT_NAME}_core)
  target_include_directories(test_pointcloud_validation PRIVATE include)
  ament_add_test(test_mavlink_sender_logic
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mavlink_sender_logic.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(test_config_layering
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_config_layering.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_test(test_replay_utils
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_replay_utils.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(rosidl_default_runtime)

ament_package()

