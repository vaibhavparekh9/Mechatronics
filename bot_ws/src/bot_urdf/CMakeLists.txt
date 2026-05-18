cmake_minimum_required(VERSION 3.8)
project(bot_urdf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf meshes config launch worlds maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/cmd_vel_to_odom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
