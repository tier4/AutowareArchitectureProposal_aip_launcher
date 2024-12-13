cmake_minimum_required(VERSION 3.5)
project(aip_xx1_description)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

find_package(PythonInterp REQUIRED) # cspell: ignore Interp
# Specify the path to your Python script
set(PYTHON_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/compile_xacro.py")
set(PYTHON_TEMPLATE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/templates")
set(PYTHON_CALIBRATION_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
set(PYTHON_XACRO_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/urdf")

# Create a custom command to run the Python script

add_custom_target(xacro_compilation ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/python_script_run_flag # this target will ask for `python_script_run_flag`
)

add_custom_command(
    # this command will claim to create `python_script_run_flag`, but will not actually do so; only running the script
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/python_script_run_flag
    COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_SCRIPT} ${PYTHON_TEMPLATE_DIRECTORY} ${PYTHON_CALIBRATION_DIRECTORY} ${PYTHON_XACRO_DIRECTORY} ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    DEPENDS ${PYTHON_SCRIPT} # Add dependencies here
    COMMENT "Running Python script for URDF creation"
)

ament_auto_package(INSTALL_TO_SHARE
    urdf
    config
)
