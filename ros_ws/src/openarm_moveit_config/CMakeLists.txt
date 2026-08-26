cmake_minimum_required(VERSION 3.8)
project(openarm_moveit_config)

find_package(ament_cmake REQUIRED)

# 安装配置文件
install(
  DIRECTORY config launch srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
