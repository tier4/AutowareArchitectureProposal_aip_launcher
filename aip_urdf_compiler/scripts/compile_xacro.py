import enum
import functools
import os
from typing import Dict

from jinja2 import Template
import yaml


def load_yaml(file_path: str) -> Dict:
    with open(file_path, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None


class Transformation:
    def __init__(self, transformation: Dict, base_frame: str, child_frame: str):
        try:
            self.x = transformation["x"]
            self.y = transformation["y"]
            self.z = transformation["z"]
            self.roll = transformation["roll"]
            self.pitch = transformation["pitch"]
            self.yaw = transformation["yaw"]
            self.base_frame = base_frame
            self.child_frame = child_frame
            self.type: str = transformation.get("type", "")

            self.name = self.child_frame.replace("_base_link", "").replace("_link", "")

        except KeyError as e:
            print(f"Error: Key {e} not in transformation dictionary")
            raise e

    def serialize_single(self, key: str) -> str:
        return f"${{calibration['{self.base_frame}']['{self.child_frame}']['{key}']}}"

    def serialize(self) -> str:
        return f"""
        name=\"{self.name}\"
        parent=\"{self.base_frame}\"
        x=\"{self.serialize_single('x')}\"
        y=\"{self.serialize_single('y')}\"
        z=\"{self.serialize_single('z')}\"
        roll=\"{self.serialize_single('roll')}\"
        pitch=\"{self.serialize_single('pitch')}\"
        yaw=\"{self.serialize_single('yaw')}\"
        """


class Calibration:
    def __init__(self, calibration: Dict):
        self.base_dict: Dict = calibration
        assert len(calibration.keys()) == 1, "Calibration file should have only one base frame"
        assert isinstance(
            list(calibration.keys())[0], str
        ), "Calibration file should have only one base frame with key as a string"
        self.base_frame: str = list(calibration.keys())[0]

        assert isinstance(
            calibration[self.base_frame], dict
        ), "Calibration file should have only one base frame with value as a dictionary"

        self.transforms: Dict[str, Transformation] = {}

        for key in calibration[self.base_frame]:
            assert isinstance(key, str), "child frames should be strings"
            try:
                self.transforms[key] = Transformation(
                    calibration[self.base_frame][key], self.base_frame, key
                )
            except KeyError as e:
                print(f"Error: Key {e} not in calibration dictionary of {key}")
                raise e


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
    VELODYNE16 = "VLP-16.urdf"
    VLS128 = "VLS-128.urdf"
    RADAR = "radar"
    JOINT_UNITS = "units"


def obtain_link_type(link: Transformation) -> LinkType:
    if len(link.type) > 0:
        # use explicit type string to obtain link
        link_type_lower = link.type.lower()

        # Check each enum value for a match
        for type_enum in LinkType:
            if link_type_lower == type_enum.value.lower():
                return type_enum
    # if there is no match, or the type is not defined:
    return determine_link_type(link.child_frame)


def determine_link_type(link_name: str) -> LinkType:
    if "cam" in link_name:
        return LinkType.CAMERA

    if "imu" in link_name or "gnss" in link_name:
        return LinkType.IMU

    if "livox" in link_name:
        return LinkType.LIVOX

    if "velodyne" in link_name:
        if "top" in link_name:
            return LinkType.VLS128
        else:
            return LinkType.VELODYNE16

    if "radar" in link_name or "ars" in link_name:
        return LinkType.RADAR

    if "pandar_40p" in link_name:
        return LinkType.PANDAR_40P

    if "pandar_qt" in link_name:
        return LinkType.PANDAR_QT

    if "hesai_top" in link_name:
        return LinkType.PANDAR_OT128

    if "hesai_front" in link_name:
        return LinkType.PANDAR_XT32

    if "hesai" in link_name:
        return LinkType.PANDAR_XT32

    else:
        print(f"Link type not found for {link_name}, suspected to be a joint unit")
        return LinkType.JOINT_UNITS


BASE_STRING = """<xacro:{type}
        name=\"{child_frame}\"
        parent=\"{base_frame}\"
        x=\"{x}\"
        y=\"{y}\"
        z=\"{z}\"
        roll=\"{roll}\"
        pitch=\"{pitch}\"
        yaw=\"{yaw}\"
        {extra}
    />"""

VLD_STRING = """<xacro:{type} parent=\"{base_frame}\" name=\"{child_frame}\" topic=\"/points_raw\" hz=\"10\" samples=\"220\" gpu=\"$(arg gpu)\">
    <origin
        xyz=\"{x}
            {y}
            {z}\"
        rpy=\"{roll}
            {pitch}
            {yaw}\"
    />
    </xacro:{type}>"""


def base_string_func(macro_type: str, transform: Transformation) -> str:
    if macro_type == "monocular_camera_macro":
        extra = """fps=\"30\"
        width=\"800\"
        height=\"400\"
        namespace=\"\"
        fov=\"1.3\""""
    elif macro_type == "imu_macro":
        extra = """fps=\"100\"
        namespace=\"\""""
    else:
        extra = ""
    if "pandar" in transform.type or "livox" in transform.type or "camera" in transform.type:
        # In commen sensor description, LiDAR and camera macros will automatically be attached with a "base_link" name.
        # So we need to strip away
        child_frame = transform.name.replace("_base_link", "").replace("_link", "")
    else:
        child_frame = transform.child_frame
    return BASE_STRING.format(
        type=macro_type,
        base_frame=transform.base_frame,
        child_frame=child_frame,  # pandar
        x=transform.serialize_single("x"),
        y=transform.serialize_single("y"),
        z=transform.serialize_single("z"),
        roll=transform.serialize_single("roll"),
        pitch=transform.serialize_single("pitch"),
        yaw=transform.serialize_single("yaw"),
        extra=extra,
    )


def VLP16_func(transform: Transformation) -> str:
    return VLD_STRING.format(
        type="VLP-16",
        base_frame=transform.base_frame,
        child_frame=transform.name,
        x=transform.serialize_single("x"),
        y=transform.serialize_single("y"),
        z=transform.serialize_single("z"),
        roll=transform.serialize_single("roll"),
        pitch=transform.serialize_single("pitch"),
        yaw=transform.serialize_single("yaw"),
    )


def VLS128_func(transform: Transformation) -> str:
    return VLD_STRING.format(
        type="VLS-128",
        base_frame=transform.base_frame,
        child_frame=transform.name,
        x=transform.serialize_single("x"),
        y=transform.serialize_single("y"),
        z=transform.serialize_single("z"),
        roll=transform.serialize_single("roll"),
        pitch=transform.serialize_single("pitch"),
        yaw=transform.serialize_single("yaw"),
    )


link_dicts = {
    LinkType.CAMERA: {
        "including_file": "$(find camera_description)/urdf/monocular_camera.xacro",
        "string_api": functools.partial(base_string_func, "monocular_camera_macro"),
    },
    LinkType.IMU: {
        "including_file": "$(find imu_description)/urdf/imu.xacro",
        "string_api": functools.partial(base_string_func, "imu_macro"),
    },
    LinkType.VELODYNE16: {
        "including_file": "$(find velodyne_description)/urdf/VLP-16.urdf.xacro",
        "string_api": VLP16_func,
    },
    LinkType.VLS128: {
        "including_file": "$(find vls_description)/urdf/VLS-128.urdf.xacro",
        "string_api": VLS128_func,
    },
    LinkType.PANDAR_40P: {
        "including_file": "$(find pandar_description)/urdf/pandar_40p.xacro",
        "string_api": functools.partial(base_string_func, "Pandar40P"),
    },
    LinkType.PANDAR_OT128: {
        "including_file": "$(find pandar_description)/urdf/pandar_ot128.xacro",
        "string_api": functools.partial(base_string_func, "PandarOT-128"),
    },
    LinkType.PANDAR_XT32: {
        "including_file": "$(find pandar_description)/urdf/pandar_xt32.xacro",
        "string_api": functools.partial(base_string_func, "PandarXT-32"),
    },
    LinkType.PANDAR_QT: {
        "including_file": "$(find pandar_description)/urdf/pandar_qt.xacro",
        "string_api": functools.partial(base_string_func, "PandarQT"),
    },
    LinkType.PANDAR_QT128: {
        "including_file": "$(find pandar_description)/urdf/pandar_qt128.xacro",
        "string_api": functools.partial(base_string_func, "PandarQT-128"),
    },
    LinkType.LIVOX: {
        "including_file": "$(find livox_description)/urdf/livox_horizon.xacro",
        "string_api": functools.partial(base_string_func, "livox_horizon_macro"),
    },
    LinkType.RADAR: {
        "including_file": "$(find radar_description)/urdf/radar.xacro",
        "string_api": functools.partial(base_string_func, "radar_macro"),
    },
    LinkType.JOINT_UNITS: {
        "including_file": "{filename}.xacro",
    },
}


def main(
    template_directory: str,
    calibration_directory: str,
    output_directory: str,
    project_name: str,
):
    os.makedirs(output_directory, exist_ok=True)
    # Load the template
    with open(os.path.join(template_directory, "sensors.xacro.template"), "r") as file:
        base_template = Template(file.read())

    # Render the template
    calibration_path = os.path.join(calibration_directory, "sensors_calibration.yaml")
    calib_yaml = load_yaml(calibration_path)
    calib = Calibration(calib_yaml)

    render_meta_data = {}
    render_meta_data["default_config_path"] = f"$(find {project_name})/config"
    render_meta_data["sensor_calibration_yaml_path"] = "$(arg config_dir)/sensors_calibration.yaml"
    render_meta_data["sensor_units_includes"] = []
    render_meta_data["sensor_units"] = []
    render_meta_data["isolated_sensors_includes"] = []
    render_meta_data["isolated_sensors"] = []

    include_text = set()
    sensor_items = []
    for _, transform in calib.transforms.items():
        link_type: LinkType = obtain_link_type(transform)
        if link_type == LinkType.JOINT_UNITS:
            render_meta_data["sensor_units_includes"].append(
                link_dicts[link_type]["including_file"].format(filename=transform.name)
            )
            render_meta_data["sensor_units"].append(
                {
                    "base_frame": transform.base_frame,
                    "child_frame": transform.child_frame,
                    "macro_name": f"{transform.name}_macro",
                    "name": transform.name,
                }
            )
        else:
            include_text.add(link_dicts[link_type]["including_file"])
            sensor_items.append(link_dicts[link_type]["string_api"](transform))

    render_meta_data["isolated_sensors_includes"] = list(include_text)
    render_meta_data["isolated_sensors"] = sensor_items

    rendered = base_template.render(render_meta_data)
    print(rendered)

    print("=====================================")
    # Save the rendered template
    with open(os.path.join(output_directory, "sensors.xacro"), "w") as file:
        file.write(rendered)

    # Write Sensor Units into separate files
    with open(os.path.join(template_directory, "sensor_unit.xacro.template"), "r") as file:
        sensor_units_template = Template(file.read())

    for i, sensor_unit in enumerate(render_meta_data["sensor_units"]):
        sensor_unit_calib_path = os.path.join(
            calibration_directory, f"{sensor_unit['name']}_calibration.yaml"
        )
        sensor_unit_calib_yaml = load_yaml(sensor_unit_calib_path)
        sensor_unit_calib = Calibration(sensor_unit_calib_yaml)
        sensor_unit_render_meta_data = {}
        sensor_unit_render_meta_data["unit_macro_name"] = sensor_unit["macro_name"]
        sensor_unit_render_meta_data["default_config_path"] = render_meta_data[
            "default_config_path"
        ]
        sensor_unit_render_meta_data["joint_unit_name"] = sensor_unit["name"]
        sensor_unit_render_meta_data["current_base_link"] = sensor_unit_calib.base_frame
        sensor_unit_isolated_sensors = []
        for _, transform in sensor_unit_calib.transforms.items():
            link_type: LinkType = obtain_link_type(transform)
            include_text.add(link_dicts[link_type]["including_file"])
            print(transform.name)
            sensor_unit_isolated_sensors.append(link_dicts[link_type]["string_api"](transform))
        sensor_unit_render_meta_data["isolated_sensors_includes"] = list(include_text)
        sensor_unit_render_meta_data["isolated_sensors"] = sensor_unit_isolated_sensors

        rendered = sensor_units_template.render(sensor_unit_render_meta_data)
        print(rendered)
        with open(os.path.join(output_directory, f'{sensor_unit["name"]}.xacro'), "w") as file:
            file.write(rendered)
        print("=====================================")

    return 0


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Process four positional arguments.")

    # Add four positional arguments
    parser.add_argument("template_directory", type=str, help="The first argument")
    parser.add_argument("calibration_directory", type=str, help="The second argument")
    parser.add_argument("output_directory", type=str, help="The third argument")
    parser.add_argument("project_name", type=str, help="The fourth argument")

    # Parse the arguments
    args = parser.parse_args()

    main(
        args.template_directory,
        args.calibration_directory,
        args.output_directory,
        args.project_name,
    )
