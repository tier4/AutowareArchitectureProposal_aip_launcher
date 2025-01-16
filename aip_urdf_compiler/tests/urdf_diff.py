#!/usr/bin/python3
"""
URDF Model Difference Analyzer.

This script analyzes and compares two XML/Xacro files, specifically designed for ROS URDF/Xacro files
containing sensor configurations. It identifies and reports differences in included files, sensor configurations,
and parameter changes between two versions of a file.

The analyzer is particularly useful for:
- Tracking changes in sensor configurations
- Validating URDF/Xacro file modifications
- Documenting sensor setup modifications
- Quality assurance of robot configuration changes

Author: [Your Name]
Date: [Current Date]
"""

import argparse
from collections import OrderedDict
from typing import Dict
from typing import List
import xml.etree.ElementTree as ET


class XacroAnalyzer:
    """
    A class to analyze differences between two Xacro/XML files.

    This class provides functionality to compare two Xacro files and identify changes
    in includes, sensor configurations, and parameters.

    Attributes:
        original_text (str): Content of the original file
        new_text (str): Content of the new file
        original_root (ET.Element): XML tree root of the original file
        new_root (ET.Element): XML tree root of the new file
        ns (dict): Namespace dictionary for Xacro XML parsing
    """

    def __init__(self, original_file: str, new_file: str):
        """
        Initialize the XacroAnalyzer with two files to compare.

        Args:
            original_file (str): Path to the original Xacro file
            new_file (str): Path to the new Xacro file
        """
        self.original_text = self._read_file(original_file)
        self.new_text = self._read_file(new_file)
        self.original_root = ET.fromstring(self.original_text)
        self.new_root = ET.fromstring(self.new_text)
        self.ns = {"xacro": "http://ros.org/wiki/xacro"}

    @staticmethod
    def _read_file(filename: str) -> str:
        """
        Read content from a file.

        Args:
            filename (str): Path to the file to read

        Returns:
            str: Content of the file
        """
        with open(filename, "r", encoding="utf-8") as f:
            return f.read()

    def _get_includes(self, root: ET.Element) -> List[str]:
        """
        Extract all include statements from an XML root.

        Args:
            root (ET.Element): XML root element to search

        Returns:
            List[str]: Sorted list of included filenames
        """
        includes = []
        for include in root.findall(".//xacro:include", self.ns):
            includes.append(include.get("filename"))
        return sorted(includes)

    def _sort_params(self, params: Dict) -> OrderedDict:
        """
        Sort parameters in a standardized order with common parameters first.

        Args:
            params (Dict): Dictionary of parameters to sort

        Returns:
            OrderedDict: Sorted parameters with common parameters first
        """
        common_params = ["name", "parent", "x", "y", "z", "roll", "pitch", "yaw"]
        sorted_params = OrderedDict()

        # Add common params in specific order
        for param in common_params:
            if param in params:
                sorted_params[param] = params[param]

        # Add remaining params alphabetically
        for param in sorted(params.keys()):
            if param not in common_params:
                sorted_params[param] = params[param]

        return sorted_params

    def _get_sensors(self, root: ET.Element) -> Dict[str, List[Dict]]:
        """
        Extract all sensor configurations from an XML root.

        Args:
            root (ET.Element): XML root element to search

        Returns:
            Dict[str, List[Dict]]: Dictionary of sensor types mapping to their configurations
        """
        sensors = {"cameras": [], "lidars": [], "imus": [], "radars": []}

        sensor_patterns = {
            "cameras": ["camera", "monocular"],
            "lidars": ["hesai", "velodyne", "pandar", "lidar"],
            "imus": ["imu", "gnss"],
            "radars": ["radar"],
        }

        for macro in root.findall(".//*[@name]"):
            name = macro.get("name", "")
            params = dict(macro.attrib)

            sensor_type = None
            for type_name, patterns in sensor_patterns.items():
                if any(pattern in name.lower() for pattern in patterns):
                    sensor_type = type_name
                    break

            if sensor_type:
                sensors[sensor_type].append({"name": name, "params": self._sort_params(params)})

        for sensor_type in sensors:
            sensors[sensor_type].sort(key=lambda x: x["name"])
        return sensors

    def _format_params(self, params: OrderedDict) -> str:
        """
        Format parameters for readable output.

        Args:
            params (OrderedDict): Parameters to format

        Returns:
            str: Formatted parameter string
        """
        return "\n".join([f'    {k}="{v}"' for k, v in params.items()])

    def analyze_differences(self) -> str:
        """
        Analyze and report differences between the two Xacro files.

        This method performs a comprehensive comparison of:
        - Included files
        - Sensor configurations
        - Parameter changes

        Returns:
            str: Formatted report of all differences found
        """
        output = []

        # 1. Compare includes
        output.append("# Key Differences\n")
        output.append("## 1. Include Files")
        original_includes = self._get_includes(self.original_root)
        new_includes = self._get_includes(self.new_root)

        added_includes = set(new_includes) - set(original_includes)
        removed_includes = set(original_includes) - set(new_includes)

        if added_includes or removed_includes:
            output.append("### Changes:")
            if added_includes:
                output.append("**Added:**")
                for inc in sorted(added_includes):
                    output.append(f"- {inc}")
            if removed_includes:
                output.append("**Removed:**")
                for inc in sorted(removed_includes):
                    output.append(f"- {inc}")
            output.append("")

        # 2. Compare sensors
        original_sensors = self._get_sensors(self.original_root)
        new_sensors = self._get_sensors(self.new_root)

        output.append("## 2. Sensor Configuration Changes")

        for sensor_type in ["cameras", "lidars", "imus", "radars"]:
            orig_sensor = original_sensors[sensor_type]
            new_sensor = new_sensors[sensor_type]

            if orig_sensor or new_sensor:
                output.append(f"\n### {sensor_type.title()}")

                # Compare sensor names
                orig_names = [s["name"] for s in orig_sensor]
                new_names = [s["name"] for s in new_sensor]

                if orig_names != new_names:
                    output.append("#### Name Changes:")
                    output.append(
                        f"**Original ({len(orig_names)})**: " + ", ".join(sorted(orig_names))
                    )
                    output.append(f"**New ({len(new_names)})**: " + ", ".join(sorted(new_names)))

                # Compare parameters
                if orig_sensor and new_sensor:
                    output.append("\n#### Parameter Changes:")

                    # Compare parameters of first sensor of each type
                    orig_params = orig_sensor[0]["params"]
                    new_params = new_sensor[0]["params"]

                    added_params = set(new_params.keys()) - set(orig_params.keys())
                    removed_params = set(orig_params.keys()) - set(new_params.keys())
                    changed_params = {
                        k: (orig_params[k], new_params[k])
                        for k in set(orig_params.keys()) & set(new_params.keys())
                        if orig_params[k] != new_params[k]
                    }

                    if added_params:
                        output.append("**Added parameters:**")
                        for param in sorted(added_params):
                            output.append(f'- {param}: "{new_params[param]}"')

                    if removed_params:
                        output.append("**Removed parameters:**")
                        for param in sorted(removed_params):
                            output.append(f'- {param}: "{orig_params[param]}"')

                    if changed_params:
                        output.append("**Modified parameters:**")
                        for param, (old_val, new_val) in sorted(changed_params.items()):
                            output.append(f'- {param}: "{old_val}" → "{new_val}"')

        return "\n".join(output)


def main():
    # Create argument parser
    parser = argparse.ArgumentParser(
        description="Compare two XACRO files and analyze their differences"
    )

    # Add arguments
    parser.add_argument(
        "--original", "-o", required=True, help="Path to the original sensors.xacro file"
    )

    parser.add_argument("--new", "-n", required=True, help="Path to the new sensors.xacro file")

    # Parse arguments
    args = parser.parse_args()

    # Create analyzer instance with provided file paths
    analyzer = XacroAnalyzer(args.original, args.new)

    # Print analysis results
    print(analyzer.analyze_differences())


if __name__ == "__main__":
    main()
