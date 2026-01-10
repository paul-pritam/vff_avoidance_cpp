cmake_minimum_required(VERSION 3.8)
project(vff_avoidance) # Matches your root folder name

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Ingredients
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# 2. Preparation
include_directories(include)

# 3. Cooking the Library
# Matches: src/vff_avoidance/avoidance_node.cpp
add_library(${PROJECT_NAME} SHARED 
  src/vff_avoidance/avoidance_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  visualization_msgs
)

# 4. Cooking the Executable
# Matches: src/vff_main.cpp
add_executable(avoidance_vff_node src/vff_main.cpp)

target_link_libraries(avoidance_vff_node ${PROJECT_NAME})
ament_target_dependencies(avoidance_vff_node 
  rclcpp 
  geometry_msgs 
  sensor_msgs 
  visualization_msgs
)

# 5. Plating (Install)
install(TARGETS 
  ${PROJECT_NAME}
  avoidance_vff_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 6. Testing (Phase 7 Prep)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # We will write this file next!
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(vff_test tests/vff_test.cpp)
  target_link_libraries(vff_test ${PROJECT_NAME})
  ament_target_dependencies(vff_test rclcpp sensor_msgs geometry_msgs)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp geometry_msgs sensor_msgs visualization_msgs)

ament_package()