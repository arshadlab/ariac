cmake_minimum_required(VERSION 3.5)
project(osrf_gear)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override")
endif()

if(WIN32)
  add_compile_definitions(
    # For math constants
    _USE_MATH_DEFINES
    # Minimize Windows namespace collision
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(ariac_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(gazebo REQUIRED)




###########
## Build ##
###########

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Create the libVacuumGripperPlugin.so library.
set(vacuum_gripper_plugin_name VacuumGripperPlugin)
add_library(${vacuum_gripper_plugin_name} src/VacuumGripperPlugin.cc)
target_include_directories(${vacuum_gripper_plugin_name}  PUBLIC include)
ament_target_dependencies(${vacuum_gripper_plugin_name}
  "gazebo_ros"
  "gazebo_dev"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)

ament_export_libraries(${vacuum_gripper_plugin_name})

# Create the ROSlibVacuumGripperPlugin.so library.
set(conveyorbest_plugin_name ConveyorBeltPlugin)
add_library(${conveyorbest_plugin_name} SHARED src/ConveyorBeltPlugin.cc)
target_include_directories(${conveyorbest_plugin_name}  PUBLIC include)
ament_target_dependencies(${conveyorbest_plugin_name}
  "gazebo_ros"
  "gazebo_dev"
  "ariac_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)

ament_export_libraries(${conveyorbelt_plugin_name})


# Create the ROSlibVacuumGripperPlugin.so library.
#set(rosconveyorbest_plugin_name ROSConveyorBeltPlugin)
add_library(ROSConveyorBeltPlugin SHARED src/ROSConveyorBeltPlugin.cc)
target_include_directories(ROSConveyorBeltPlugin PUBLIC include)
ament_target_dependencies(ROSConveyorBeltPlugin
  "gazebo_ros"
  "gazebo_dev"
  "ariac_msgs"
  "rclcpp"
  "std_msgs"
  "std_srvs"
)


ament_export_libraries(ROSConveyorBeltPlugin)



ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

#############
## Install ##
#############
  
install(TARGETS
${conveyorbest_plugin_name}
ROSConveyorBeltPlugin
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)