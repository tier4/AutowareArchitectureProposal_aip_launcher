


macro(aip_cmake_urdf_compile)
    # Set the correct paths
    find_package(PythonInterp REQUIRED) # cspell: ignore Interp
    set(aip_urdf_compiler_BASE_DIR "${aip_urdf_compiler_DIR}/../")
    set(PYTHON_SCRIPT "${aip_urdf_compiler_BASE_DIR}/scripts/compile_urdf.py")
    set(PYTHON_TEMPLATE_DIRECTORY "${aip_urdf_compiler_BASE_DIR}/templates")
    set(PYTHON_CALIBRATION_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
    set(PYTHON_XACRO_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/urdf")

    message(STATUS "PYTHON_SCRIPT path: ${PYTHON_SCRIPT}")
    message(STATUS "PYTHON_TEMPLATE_DIRECTORY path: ${PYTHON_TEMPLATE_DIRECTORY}")

    # Verify that the required files exist
    if(NOT EXISTS "${PYTHON_SCRIPT}")
        message(FATAL_ERROR "Could not find compile_urdf.py at ${PYTHON_SCRIPT}")
    endif()

    # Create a custom command to run the Python script
    add_custom_target(xacro_compilation ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/python_script_run_flag
    )

    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/python_script_run_flag
        COMMAND ${PYTHON_EXECUTABLE} ${PYTHON_SCRIPT} ${PYTHON_TEMPLATE_DIRECTORY} ${PYTHON_CALIBRATION_DIRECTORY} ${PYTHON_XACRO_DIRECTORY} ${PROJECT_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${PYTHON_SCRIPT}
        COMMENT "Running Python script for URDF creation"
    )
endmacro()
