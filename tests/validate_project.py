#!/usr/bin/env python3
"""Static, ROS-independent checks for the ARM1.5 + O7 integration artifacts."""

from __future__ import annotations

import argparse
import hashlib
import math
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


PROJECT = Path(__file__).resolve().parents[1]
SRC = PROJECT / "src"
DESCRIPTION = SRC / "arm_o7_description"
XACRO_FILE = DESCRIPTION / "urdf" / "arm_o7.urdf.xacro"

ARM_JOINTS = [f"arm_joint_{index}" for index in range(1, 7)]
HAND_JOINTS = [
    "thumb_cmc_roll",
    "thumb_cmc_yaw",
    "thumb_cmc_pitch",
    "index_mcp_pitch",
    "middle_mcp_pitch",
    "ring_mcp_pitch",
    "pinky_mcp_pitch",
]
ACTUATED_JOINTS = ARM_JOINTS + HAND_JOINTS
SIM_COUPLED_JOINTS = ["thumb_mcp", "thumb_ip"]
EXPECTED_LIMITS = {
    "arm_joint_1": (-3.14, 3.14),
    "arm_joint_2": (-1.57, 1.57),
    "arm_joint_3": (-1.30, 1.30),
    "arm_joint_4": (-3.14, 3.14),
    "arm_joint_5": (-1.30, 1.30),
    "arm_joint_6": (-3.14, 3.14),
    "thumb_cmc_roll": (0.0, 1.1339),
    "thumb_cmc_yaw": (0.0, 1.9189),
    "thumb_cmc_pitch": (0.0, 0.5146),
    "index_mcp_pitch": (0.0, 1.3607),
    "middle_mcp_pitch": (0.0, 1.3607),
    "ring_mcp_pitch": (0.0, 1.3607),
    "pinky_mcp_pitch": (0.0, 1.3607),
}


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def expand(xacro_executable: str, hardware_type: str) -> ET.Element:
    command = [
        xacro_executable,
        str(XACRO_FILE),
        f"hardware_type:={hardware_type}",
    ]
    if hardware_type == "sim":
        command.extend(["controllers_file:=/tmp/sim_controllers.yaml", "ros_namespace:=/sim"])
    result = subprocess.run(command, check=True, capture_output=True)
    return ET.fromstring(result.stdout)


def validate_urdf(root: ET.Element) -> None:
    require(root.tag == "robot" and root.get("name") == "arm_o7", "unexpected robot name")
    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    require(len(links) == 26, f"expected 26 links, got {len(links)}")
    require(len(joints) == 25, f"expected 25 joints, got {len(joints)}")
    require(len(links) == len(set(links)), "duplicate link name")
    require(len(joints) == len(set(joints)), "duplicate joint name")
    require(not set(links).intersection(joints), "a link and joint share a name")

    child_links = set()
    adjacency: dict[str, list[str]] = {name: [] for name in links}
    for name, joint in joints.items():
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        require(parent in links, f"{name}: missing parent link {parent}")
        require(child in links, f"{name}: missing child link {child}")
        require(child not in child_links, f"{child}: multiple parent joints")
        child_links.add(child)
        adjacency[parent].append(child)

    roots = set(links) - child_links
    require(roots == {"world"}, f"expected root world, got {sorted(roots)}")
    visited = set()
    stack = ["world"]
    while stack:
        link = stack.pop()
        require(link not in visited, f"cycle detected at {link}")
        visited.add(link)
        stack.extend(adjacency[link])
    require(visited == set(links), "robot tree is disconnected")

    actuated = []
    for name, joint in joints.items():
        if joint.get("type") == "fixed" or joint.find("mimic") is not None:
            continue
        actuated.append(name)
        limit = joint.find("limit")
        require(limit is not None, f"{name}: missing limit")
        lower, upper = float(limit.get("lower")), float(limit.get("upper"))
        require(lower < upper, f"{name}: invalid position limit")
        require(float(limit.get("effort")) > 0.0, f"{name}: zero effort limit")
        require(float(limit.get("velocity")) > 0.0, f"{name}: zero velocity limit")
        expected = EXPECTED_LIMITS[name]
        require(
            math.isclose(lower, expected[0], abs_tol=1e-9)
            and math.isclose(upper, expected[1], abs_tol=1e-9),
            f"{name}: model limit drifted from source",
        )
    require(set(actuated) == set(ACTUATED_JOINTS), f"unexpected actuated joints: {actuated}")

    for name, joint in joints.items():
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        source = mimic.get("joint")
        require(source in joints, f"{name}: missing mimic source {source}")
        require(joints[source].find("mimic") is None, f"{name}: chained mimic is unsupported")
        multiplier = float(mimic.get("multiplier", "1"))
        offset = float(mimic.get("offset", "0"))
        source_limit = joints[source].find("limit")
        target_limit = joint.find("limit")
        mapped = [
            float(source_limit.get("lower")) * multiplier + offset,
            float(source_limit.get("upper")) * multiplier + offset,
        ]
        require(
            min(mapped) >= float(target_limit.get("lower")) - 2e-4
            and max(mapped) <= float(target_limit.get("upper")) + 2e-4,
            f"{name}: mimic range exceeds its joint limit",
        )

    # The source model's zero-configuration flange pose is a useful regression invariant.
    zero_chain = [joints[name] for name in ARM_JOINTS]
    require(
        all(joint.find("origin").get("rpy") == "0 0 0" for joint in zero_chain),
        "arm zero-chain RPY changed; update the FK regression check",
    )
    flange_xyz = [
        sum(float(joint.find("origin").get("xyz").split()[axis]) for joint in zero_chain)
        for axis in range(3)
    ]
    expected_flange = (-0.000141266240365548, 0.0001378456724126141, 0.7083)
    require(
        all(math.isclose(actual, expected, abs_tol=1e-9) for actual, expected in zip(flange_xyz, expected_flange)),
        f"unexpected zero-pose flange translation: {flange_xyz}",
    )

    for mesh in root.findall(".//mesh"):
        uri = mesh.get("filename")
        prefix = "package://arm_o7_description/"
        require(uri.startswith(prefix), f"non-package mesh URI: {uri}")
        target = DESCRIPTION / uri.removeprefix(prefix)
        require(target.is_file(), f"missing mesh: {target}")
    collision_uris = [mesh.get("filename") for mesh in root.findall(".//collision/geometry/mesh")]
    require(collision_uris, "no collision meshes")
    require(
        all("_collision/" in uri for uri in collision_uris),
        "visual high-resolution mesh is being used for collision",
    )


def validate_sim_control(sim_root: ET.Element) -> None:
    control = sim_root.find("ros2_control")
    require(control is not None, "sim expansion lacks ros2_control")
    plugin = control.find("hardware/plugin")
    require(plugin is not None and plugin.text == "gz_ros2_control/GazeboSimSystem", "wrong sim plugin")
    control_joints = {joint.get("name"): joint for joint in control.findall("joint")}
    require(set(ACTUATED_JOINTS).issubset(control_joints), "missing command joints")
    for name, joint in control_joints.items():
        commands = joint.findall("command_interface")
        if name in ACTUATED_JOINTS:
            require([item.get("name") for item in commands] == ["position"], f"{name}: bad command interface")
        elif name in SIM_COUPLED_JOINTS:
            require(
                [item.get("name") for item in commands] == ["position"],
                f"{name}: missing simulation coupling command interface",
            )
            require(joint.get("mimic") == "false", f"{name}: automatic mimic must be disabled in simulation")
        else:
            require(not commands, f"mimic joint {name} must not be commanded")


def validate_yaml_and_srdf() -> None:
    controllers = yaml.safe_load((DESCRIPTION / "config" / "sim_controllers.yaml").read_text())
    require(
        controllers["/**/arm_controller"]["ros__parameters"]["joints"] == ARM_JOINTS,
        "arm controller joint list/order mismatch",
    )
    require(
        controllers["/**/hand_controller"]["ros__parameters"]["joints"] == HAND_JOINTS,
        "hand controller joint list/order mismatch",
    )
    require(
        controllers["/**/thumb_coupling_controller"]["ros__parameters"]["joints"]
        == SIM_COUPLED_JOINTS,
        "thumb coupling controller joint list/order mismatch",
    )
    require(
        controllers["/**/controller_manager"]["ros__parameters"]["enforce_command_limits"] is True,
        "controller_manager command limit enforcement is off",
    )

    srdf_path = SRC / "arm_o7_moveit_config" / "config" / "arm_o7.srdf"
    srdf = ET.parse(srdf_path).getroot()
    require(srdf.get("name") == "arm_o7", "SRDF robot name mismatch")
    srdf_joint_names = {item.get("name") for item in srdf.findall(".//joint")}
    require(set(ACTUATED_JOINTS).issubset(srdf_joint_names), "SRDF omits actuated joints")

    named_states = {}
    for state in srdf.findall("group_state"):
        key = (state.get("group"), state.get("name"))
        named_states[key] = {
            joint.get("name"): float(joint.get("value"))
            for joint in state.findall("joint")
        }

    expected_states = {
        ("arm", "home"),
        ("arm", "ready"),
        ("arm", "left_demo"),
        ("hand", "open"),
        ("hand", "pregrasp"),
        ("hand", "four_finger_fist"),
        ("hand", "middle_finger"),
        ("hand", "contact_closed"),
    }
    require(expected_states.issubset(named_states), "SRDF named motion library is incomplete")
    require(
        set(named_states[("arm", "left_demo")]) == set(ARM_JOINTS),
        "left_demo must define all arm joints",
    )
    middle_finger = named_states[("hand", "middle_finger")]
    require(
        set(middle_finger) == set(HAND_JOINTS),
        "middle_finger must define all hand command joints",
    )
    require(
        math.isclose(middle_finger["middle_mcp_pitch"], 0.001, abs_tol=1e-9)
        and middle_finger["index_mcp_pitch"] > 1.0
        and middle_finger["ring_mcp_pitch"] > 1.0
        and middle_finger["pinky_mcp_pitch"] > 1.0,
        "middle_finger gesture no longer keeps only the middle finger extended",
    )


def validate_binary_stls() -> None:
    for path in sorted((DESCRIPTION / "meshes").glob("*_collision/*.STL")):
        data = path.read_bytes()
        require(len(data) >= 84, f"invalid STL: {path}")
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        require(len(data) == 84 + 50 * triangle_count, f"non-binary or truncated STL: {path}")
        require(triangle_count > 3, f"empty collision mesh: {path}")


def validate_manifest() -> None:
    manifest_path = PROJECT / "ASSET_MANIFEST.sha256"
    require(manifest_path.is_file(), "missing ASSET_MANIFEST.sha256")
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        expected, relative = line.split("  ", 1)
        target = PROJECT / relative
        require(target.is_file(), f"manifest file missing: {relative}")
        actual = hashlib.sha256(target.read_bytes()).hexdigest()
        require(actual == expected, f"asset hash mismatch: {relative}")


def compile_python() -> None:
    for path in sorted(SRC.glob("**/*.py")):
        if "__pycache__" not in path.parts:
            compile(path.read_text(encoding="utf-8"), str(path), "exec")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--xacro", default=shutil.which("xacro"), help="path to xacro executable")
    args = parser.parse_args()
    require(args.xacro, "xacro executable not found; source /opt/ros/jazzy/setup.bash")
    validate_urdf(expand(args.xacro, "none"))
    validate_sim_control(expand(args.xacro, "sim"))
    validate_yaml_and_srdf()
    validate_binary_stls()
    validate_manifest()
    compile_python()
    print("PASS: Xacro, URDF tree, limits, mimic joints, meshes, ros2_control, controllers, SRDF, hashes, and Python syntax")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (AssertionError, KeyError, ET.ParseError, subprocess.CalledProcessError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
