cmake_minimum_required(VERSION 3.8)
project(wpt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) 
find_package(aura_msg REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(tf2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(${GeographicLib_INCLUDE_DIRS})


# Include directories
# include_directories(include)
include_directories(include ${CMAKE_CURRENT_SOURCE_DIR}/include)



# Specify Python executable
install(PROGRAMS
  # Install Python files
  scripts/plccomm.py
  scripts/plc_test.py
  scripts/heron_map_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(talker src/ex_pub.cpp)
ament_target_dependencies(talker rclcpp aura_msg)         # CHANGE

add_executable(dp src/DP.cpp)
target_include_directories(dp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(dp rclcpp aura_msg)


add_executable(listener src/ex_sub.cpp)
ament_target_dependencies(listener rclcpp aura_msg)     # CHANGE

add_executable(aura_wpt src/aura_wpt.cpp)
ament_target_dependencies(aura_wpt rclcpp aura_msg tf2 sensor_msgs)     # CHANGE
target_link_libraries(aura_wpt ${GeographicLib_LIBRARIES})

add_executable(system_id src/SI.cpp)
ament_target_dependencies(system_id rclcpp aura_msg tf2 sensor_msgs)     # CHANGE
target_link_libraries(system_id ${GeographicLib_LIBRARIES})

add_executable(ekf src/ekf_filter.cpp)
ament_target_dependencies(ekf rclcpp aura_msg tf2 sensor_msgs geometry_msgs)     # CHANGE
target_link_libraries(ekf ${GeographicLib_LIBRARIES})

# Add the executable
add_executable(wpt_publisher src/Lawnmower_pub.cpp)
# Link dependencies
ament_target_dependencies(wpt_publisher
  rclcpp
  aura_msg
  GeographicLib
)
target_link_libraries(wpt_publisher ${GeographicLib_LIBRARIES})


install(TARGETS
  talker
  listener
  aura_wpt
  wpt_publisher
  ekf
  system_id
  dp
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
