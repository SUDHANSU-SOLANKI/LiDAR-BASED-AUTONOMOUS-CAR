cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Find LibSerial (Manual Discovery)
find_library(LIBSERIAL_LIB NAMES serial REQUIRED)
find_path(LIBSERIAL_INCLUDE NAMES libserial/SerialPort.h REQUIRED)

# 2. Define the Library Target
add_library(${PROJECT_NAME} SHARED 
  src/my_robot_hardware.cpp
)

# 3. Include Directories
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  ${LIBSERIAL_INCLUDE}
)

# 4. Link ROS 2 Dependencies 
# (This uses the PLAIN signature internally in Humble)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
)

# 5. Link LibSerial using the PLAIN signature 
# (Removed the 'PRIVATE' keyword to match ament_target_dependencies)
target_link_libraries(${PROJECT_NAME}
  ${LIBSERIAL_LIB}
)

# 6. Export the Plugin
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware.xml)

# 7. Install
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_package()