cmake_minimum_required(VERSION 3.5)
project(aip_urdf_compiler)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Export the package's share directory path
set(aip_urdf_compiler_SHARE_DIR "share/${PROJECT_NAME}")
ament_auto_package(INSTALL_TO_SHARE
    templates
    scripts
)
