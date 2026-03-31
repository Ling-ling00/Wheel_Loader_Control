cmake_minimum_required(VERSION 3.5)
project(loader_sim_pkg)

################ FIND DEPENDENCIES  #################
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/linkage_node.py
  scripts/state_generator.py
  scripts/trajectory_generator.py
  scripts/kinematics_node.py
  scripts/loader_feedback_node.py
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  urdf
  launch
  meshes
  config
  world
  DESTINATION share/${PROJECT_NAME})

ament_package()
