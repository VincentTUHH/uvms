
cmake_minimum_required(VERSION 3.5)

project(uvms_mpc_ctrl)

find_package(ament_cmake REQUIRED)

find_package(ament_cmake_python REQUIRED)

find_package(rclpy REQUIRED)


install(PROGRAMS
  nodes/ltv_mpc_node.py
  nodes/wrench_decision_node.py
  nodes/thruster_decision_node.py
  nodes/joint_vel_decision_node.py
  DESTINATION lib/${PROJECT_NAME}

)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
