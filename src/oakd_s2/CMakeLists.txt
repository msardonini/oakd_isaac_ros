cmake_minimum_required(VERSION 3.8)

project(oakd_s2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(depthai_bridge REQUIRED)
find_package(depthai CONFIG REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# This line is a bit of a hack.. ament won't accept depthai::opencv so we have to explicity define where it's
# libraries are saved
set(CMAKE_BUILD_RPATH "/usr/local/lib")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(oakd_s2_src
  src/oakd_s2_src.cpp
  src/oakd.cpp
)
target_include_directories(oakd_s2_src
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(oakd_s2_src
  PUBLIC
    c_std_99
    cxx_std_17
)  # Require C99 and C++17

target_link_libraries(oakd_s2_src
  depthai::opencv
)
ament_target_dependencies(oakd_s2_src
  cv_bridge
  depthai_bridge
  depthai_ros_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS oakd_s2_src
  DESTINATION lib/${PROJECT_NAME})


add_executable(imu_frame_publisher
  src/imu_frame_publisher.cpp
)
target_include_directories(imu_frame_publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(imu_frame_publisher
  PUBLIC
    c_std_99
    cxx_std_17
)  # Require C99 and C++17

ament_target_dependencies(imu_frame_publisher
  rclcpp
  geometry_msgs
  tf2_ros
  # sensor_msgs
)
install(TARGETS imu_frame_publisher
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# # debug
# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)
# install(TARGETS
# talker
# DESTINATION lib/${PROJECT_NAME})
# # debug

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
