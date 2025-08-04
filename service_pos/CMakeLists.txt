cmake_minimum_required(VERSION 3.5)
project(service_pos)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Deklarasi service yang ingin dibuat
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetPositions.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

# Tambahkan eksekutabel
add_executable(get_positions_server src/get_positions_server.cpp)

# --- FIX START ---
# Use ament_target_dependencies for external ROS 2 library dependencies
ament_target_dependencies(get_positions_server
  rclcpp
  std_msgs
  geometry_msgs
)

# Use target_link_libraries to link against the generated typesupport for your custom service.
# This should happen AFTER rosidl_generate_interfaces.
target_link_libraries(get_positions_server
  ${PROJECT_NAME}__rosidl_typesupport_cpp # For C++ typesupport
)
# If you also use C-based typesupport or need other specific typesupports,
# you might add: ${PROJECT_NAME}__rosidl_typesupport_c
# But __rosidl_typesupport_cpp is generally sufficient for C++ nodes.
# --- FIX END ---

# Install binary
install(TARGETS
  get_positions_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install file service
install(DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_export_dependencies(
  rclcpp
  geometry_msgs
  std_msgs
  rosidl_default_runtime
)

ament_package()
