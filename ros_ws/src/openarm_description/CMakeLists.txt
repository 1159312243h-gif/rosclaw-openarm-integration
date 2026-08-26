cmake_minimum_required(VERSION 3.8)
project(openarm_description)

find_package(ament_cmake REQUIRED)

# 安装 URDF 和 mesh 文件
install(
  DIRECTORY assets
  DESTINATION share/${PROJECT_NAME}
)

# 跳过 lint 检查
set(ament_cmake_copyright_FOUND TRUE)
set(ament_cmake_cpplint_FOUND TRUE)
set(ament_cmake_lint_cmake_FOUND TRUE)
set(ament_cmake_xmllint_FOUND TRUE)
set(ament_cmake_uncrustify_FOUND TRUE)

ament_package()
