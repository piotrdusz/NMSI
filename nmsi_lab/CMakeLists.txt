cmake_minimum_required(VERSION 3.10)
project(nmsi_lab)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(a_star src/a_star.cpp)
target_link_libraries(a_star PUBLIC
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
)

target_include_directories(a_star PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(a_star PUBLIC c_std_99 cxx_std_17)

install(TARGETS a_star
DESTINATION lib/${PROJECT_NAME})

add_executable(path_follower src/path_follower.cpp)
target_link_libraries(path_follower PUBLIC
  ${rclcpp_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

target_include_directories(path_follower PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(path_follower PUBLIC c_std_99 cxx_std_17)

install(TARGETS path_follower
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  world
  DESTINATION share/${PROJECT_NAME}
)

add_executable(map_processor src/map_processor.cpp)
target_link_libraries(map_processor PUBLIC
  ${rclcpp_TARGETS}
  ${nav_msgs_TARGETS}
)
target_compile_features(map_processor PUBLIC c_std_99 cxx_std_17)

install(TARGETS map_processor
DESTINATION lib/${PROJECT_NAME})

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

ament_package()
