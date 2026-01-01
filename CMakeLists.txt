cmake_minimum_required(VERSION 3.8)
project(vff_avoidance)

# Default to C++ standard from the compiler
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

include_directories(include)

# Create shared library from your library source files
add_library(${PROJECT_NAME} SHARED
  src/vff_avoidance/vff_avoidance.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

# Create executable that uses the library
add_executable(avoidance_vff_node src/vff_main.cpp)
ament_target_dependencies(avoidance_vff_node ${dependencies})
target_link_libraries(avoidance_vff_node ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  avoidance_vff_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files so other packages can use your library
install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()