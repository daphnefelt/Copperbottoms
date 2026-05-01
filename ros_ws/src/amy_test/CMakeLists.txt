cmake_minimum_required(VERSION 3.5)
project(amy_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate ROS interfaces (messages)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TapeContour.msg"
  DEPENDENCIES std_msgs
)

# Install Python package using ament_cmake_python
# NOTE: When mixing rosidl with Python packages, avoid using ament_python_install_package
# if it conflicts with message generation targets. Install Python modules manually instead.
# ament_python_install_package(${PROJECT_NAME})

# Manually install Python modules to avoid target name conflicts
install(DIRECTORY amy_test/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install the three tape nodes as executable scripts (without .py extension)
# These are installed separately because they need to be directly executable
foreach(script tape_vision_node tape_control_node tape_recovery_node)
  install(PROGRAMS
    amy_test/${script}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${script}
  )
endforeach()


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
