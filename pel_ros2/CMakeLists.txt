cmake_minimum_required(VERSION 3.8)
project(pel_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/BuildRobotBaseGraph.srv"
  "srv/LoadGraph.srv"
  "srv/Observe.srv"
  "srv/QueryGraph.srv"
  "srv/SaveGraph.srv"
  "srv/ReasonGraph.srv"
  "srv/UpdateGraph.srv"
)

# Install the Python package itself.
install(DIRECTORY
  pel_ros2/
  DESTINATION lib/python3.10/site-packages/pel_ros2
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  pel_ros2/base_graph_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME base_graph_service
)

install(PROGRAMS
  pel_ros2/observer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME observer
)

install(PROGRAMS
  pel_ros2/observation_graph_manager.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME observation_graph_manager
)

install(PROGRAMS
  pel_ros2/first_symbolic_frame.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME first_symbolic_frame
)

install(PROGRAMS
  pel_ros2/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME main
)

ament_package()
