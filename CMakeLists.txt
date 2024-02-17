cmake_minimum_required(VERSION 3.8)
project(range_image_creator_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(VTK_INCLUDE_DIRS "/usr/include/vtk-9.1")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(VTK REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#for debug
# message(WARNING "VTK_VERSION: ${VTK_VERSION}")
# message(WARNING "VTK_INCLUDE_DIRS: ${VTK_INCLUDE_DIRS}")

include_directories(${VTK_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${TF2_ROS_INCLUDE_DIRS})

# TF base
add_executable(tf_base_range_image_creator src/tf_base_range_image_creator.cpp include/range_image_creator_node/range_image_creator.hpp)
target_link_libraries(tf_base_range_image_creator
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}  
  ${rmw_LIBRARIES}     
  ${rosidl_typesupport_interface_LIBRARIES}  
  ${rcl_interfaces_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${tf2_ros_LIBRARIES}
)

target_include_directories(tf_base_range_image_creator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  )
target_compile_features(tf_base_range_image_creator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Pose base
add_executable(pose_base_range_image_creator src/pose_base_range_image_creator.cpp)
target_link_libraries(pose_base_range_image_creator
  ${PCL_LIBRARIES}
  ${rclcpp_LIBRARIES}  
  ${rmw_LIBRARIES}     
  ${rosidl_typesupport_interface_LIBRARIES}  
  ${rcl_interfaces_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_include_directories(pose_base_range_image_creator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rosidl_typesupport_interface_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  )
target_compile_features(pose_base_range_image_creator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS tf_base_range_image_creator pose_base_range_image_creator
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    params
    map
  DESTINATION
    share/${PROJECT_NAME} 
)

install(
  DIRECTORY
    include
  DESTINATION
    include/${PROJECT_NAME}
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
