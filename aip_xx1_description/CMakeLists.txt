cmake_minimum_required(VERSION 3.5)
project(aip_xx1_description)

find_package(ament_cmake_auto REQUIRED)
find_package(aip_urdf_compiler REQUIRED)

ament_auto_find_build_dependencies()

aip_cmake_urdf_compile()

ament_auto_package(INSTALL_TO_SHARE
    urdf
    config
)
