cmake_minimum_required(VERSION 3.8)
project(dwe_camera_interfaces)

# Find CMake SCM and Python support
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find ROS IDL generator
find_package(rosidl_default_generators REQUIRED)

# Find std_msgs, as it's a dependency for CamParameters.msg
find_package(std_msgs REQUIRED)

# Generate interfaces (messages, services, actions)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CamParameters.msg"
  # List other .msg files here if you add more
  DEPENDENCIES std_msgs # Specify dependencies of your .msg files
)

# Standard Ament package registration
ament_package()