cmake_minimum_required(VERSION 3.5)
project(aip_urdf_compiler)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

# Install cmake directory
install(
  DIRECTORY cmake templates scripts
  DESTINATION share/${PROJECT_NAME}
)

# Export the package's share directory path

# Add the config extras
ament_package(
  CONFIG_EXTRAS "cmake/aip_cmake_urdf_compile.cmake"
)
