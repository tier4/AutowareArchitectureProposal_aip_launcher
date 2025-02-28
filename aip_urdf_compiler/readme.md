# aip_urdf_compiler

## Overview

The aip_urdf_compiler package provides tools for dynamically generating URDF (Unified Robot Description Format) files from configuration files during the build process. It simplifies sensor model management by automatically URDF models from sensor configurations.

## Key Features

- Dynamic URDF generation during colcon build
- Automated sensor transform processing
- Support for multiple sensor types and configurations

## Usage

### Package Integration

To use aip_urdf_compiler in your description package:

- Add the dependency in `package.xml`:

  ```xml
  <depend>aip_urdf_compiler</depend>
  ```

- Add the following to `CMakeLists.txt`:

  ```cmake
  find_package(aip_urdf_compiler REQUIRED)
  aip_cmake_urdf_compile()
  ```

- Configure your sensors in `config/sensors.yaml` with metadata values (Note: do not need to add meta values in `individual_params`):

  - `type`: Required string, corresponding to the string value from [existing sensors](#existing-sensors)
  - `frame_id`: Optional string, overwrites the TF frame ID.

- Clean up existing `.xacro` files and add to `.gitignore`:

  ```gitignore
  # In your URDF folder
  *.xacro
  ```

### Existing Sensors

```python
class LinkType(enum.Enum):
    """Enum class for the type of the link."""

    CAMERA = "monocular_camera"
    IMU = "imu"
    LIVOX = "livox_horizon"
    PANDAR_40P = "pandar_40p"
    PANDAR_OT128 = "pandar_ot128"
    PANDAR_XT32 = "pandar_xt32"
    PANDAR_QT = "pandar_qt"
    PANDAR_QT128 = "pandar_qt128"
    VELODYNE16 = "velodyne_16"
    VLS128 = "velodyne_128"
    RADAR = "radar"
    GNSS = "gnss"
    JOINT_UNITS = "units"
```

## Architecture

### Components

1. **aip_urdf_compiler**

   - Main package handling URDF generation
   - Processes configuration files
   - Manages build-time compilation

2. **aip_cmake_urdf_compile**

   - CMake macro implementation
   - Creates build targets
   - Ensures URDF regeneration on each build

3. **compile_urdf.py**
   - Configuration parser
   - Transform processor
   - URDF generator

### Compilation Process

1. **Configuration Reading**

   - Parses `config/sensors.yaml`
   - Extracts transformation data
   - Validates configurations

2. **Transform Processing**

   - Processes each sensor transform
   - Determines sensor types and frame IDs
   - Generates appropriate macro strings
   - Creates `sensors.xacro`

3. **Joint Unit Processing**
   - Handles joint unit transforms
   - Processes related YAML files
   - Generates separate URDF xacro files

## Adding New Sensors

1. Add sensor descriptions (xacro module files) in either:

   - Your target package
   - `common_sensor_description` package

2. Update the following in `compile_urdf.py`:
   - `LinkType` enumeration
   - `link_dict` mapping

## Troubleshooting

### Debug Logs

Check build logs for debugging information:

```bash
cat $workspace/log/build_<timestamp>/aip_{project}_description/streams.log
```

### Common Issues

1. Missing sensor definitions

   - Ensure sensor type is defined in `LinkType`
   - Verify xacro file exists in description package

2. TF Trees errors
   - Check frame_id values in sensors.yaml
   - Verify transform chain completeness

## Contributing

1. Follow ROS coding standards
2. Test URDF generation with various configurations
3. Update documentation for new features
