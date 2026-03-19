cmake_minimum_required(VERSION 3.8)
project(pel_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LoadGraph.srv"
  "srv/QueryGraph.srv"
  "srv/SaveGraph.srv"
  "srv/InitializeGraph.srv"
  "srv/ReasonGraph.srv"
)

# Install data files
install(DIRECTORY
  pel_ros2/
  DESTINATION lib/python3.10/site-packages/pel_ros2
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executable entry points
install(PROGRAMS
  pel_ros2/graph_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME graph_manager_node
)

install(PROGRAMS
  pel_ros2/simulator_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulator_test
)

ament_package()
