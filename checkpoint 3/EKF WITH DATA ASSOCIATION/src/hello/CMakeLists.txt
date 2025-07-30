cmake_minimum_required(VERSION 3.8)
project(hello)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)   # helper module
find_package(Eigen3 REQUIRED)                # finds Eigen 3.4+



add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp eufs_msgs geometry_msgs)
install(TARGETS node1 DESTINATION lib/${PROJECT_NAME})

add_executable(node2 src/node2.cpp)

ament_target_dependencies(node2
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
add_executable(update_ekf src/update_ekf.cpp)
ament_target_dependencies(update_ekf
  rclcpp
  eufs_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  nav_msgs
  
)
install(TARGETS update_ekf
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  node2
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(calculation src/calculation.cpp)
ament_target_dependencies(calculation eufs_msgs std_msgs sensor_msgs rclcpp geometry_msgs)
install(TARGETS calculation DESTINATION lib/${PROJECT_NAME})

add_executable(ground_truth src/ground_truth.cpp)  # Adjust name if needed
ament_target_dependencies(ground_truth
  rclcpp
  eufs_msgs
  geometry_msgs
  visualization_msgs
)
install(TARGETS
  ground_truth
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(ekf src/ekf.cpp)
ament_target_dependencies(ekf rclcpp
  geometry_msgs
  visualization_msgs
  tf2_ros 
  tf2
  tf2_geometry_msgs
  eigen3_cmake_module
  Eigen3
  eufs_msgs)

install(TARGETS
  ekf
  DESTINATION lib/${PROJECT_NAME}
)




add_executable(v src/v.cpp)
ament_target_dependencies(v rclcpp sensor_msgs)
install(TARGETS v DESTINATION lib/${PROJECT_NAME})


add_executable(actual src/actual.cpp)
ament_target_dependencies(actual rclcpp eufs_msgs visualization_msgs geometry_msgs)

install(TARGETS actual
  DESTINATION lib/${PROJECT_NAME})


add_executable(node src/node.cpp)

ament_target_dependencies(node
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME}
)


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
