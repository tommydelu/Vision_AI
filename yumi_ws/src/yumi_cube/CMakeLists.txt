cmake_minimum_required(VERSION 3.10)
project(yumi_cube)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- NODI C++ ---
add_executable(yumi_cube_node src/main.cpp src/primitives.cpp)
ament_target_dependencies(yumi_cube_node rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_geometry_msgs)
target_include_directories(yumi_cube_node PRIVATE include)

add_executable(sequence_node src/sequence.cpp)
ament_target_dependencies(sequence_node rclcpp std_msgs)

# --- INSTALLAZIONE ESEGUIBILI C++ ---
install(TARGETS
  yumi_cube_node
  sequence_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- INSTALLAZIONE NODO PYTHON ---
# Diciamo a CMake di prendere lo script Python e installarlo come eseguibile.
# L'opzione RENAME toglie l'estensione .py in modo che corrisponda esattamente
# al nome "perception_bridge_node" che hai messo nel file di launch.


# DECOMMENTA SE HAI MACOS
# install(PROGRAMS
#   scripts/locate_cube_MACOS.py
#   DESTINATION lib/${PROJECT_NAME}
#   RENAME locate_cube_node
# )

install(PROGRAMS
  scripts/locate_cube_LINUX.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME locate_cube_node
)

# --- INSTALLAZIONE CARTELLE CONDIVISE ---
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()