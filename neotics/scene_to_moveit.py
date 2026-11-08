"""Turn the arm in a Neotics Scene into everything MoveIt needs to plan for it.

    python scene_to_moveit.py samples/depot.scene.json out_dir

Writes <robot>.urdf, <robot>.srdf and move_group.yaml, a parameter file the move_group node loads directly.
"""
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml

from neotics_scene import Scene
from scene_to_urdf import link_name, to_urdf

GROUP = "arm"


def to_srdf(scene: Scene) -> str:
    robot = ET.Element("robot", name=scene.robot_name)
    group = ET.SubElement(robot, "group", name=GROUP)
    ET.SubElement(group, "chain", base_link="world", tip_link="tool")
    for label, pose in (("home", scene.home), ("reach", scene.reach)):
        state = ET.SubElement(robot, "group_state", name=label, group=GROUP)
        for name, value in zip(scene.joint_names, pose):
            ET.SubElement(state, "joint", name=name, value=f"{value:.6g}")
    links = [link_name(n) for n in scene.joint_names]
    for first, second in zip(links, links[1:]):
        ET.SubElement(robot, "disable_collisions", link1=first, link2=second, reason="Adjacent")
    ET.indent(robot)
    return ET.tostring(robot, encoding="unicode") + "\n"


def move_group_params(scene: Scene) -> dict:
    limits = {name: {"has_velocity_limits": True, "max_velocity": 2.5, "has_acceleration_limits": True, "max_acceleration": 5.0} for name in scene.joint_names}
    return {"move_group": {"ros__parameters": {
        "robot_description": to_urdf(scene),
        "robot_description_semantic": to_srdf(scene),
        "robot_description_kinematics": {GROUP: {"kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin", "kinematics_solver_timeout": 0.05}},
        "robot_description_planning": {"joint_limits": limits},
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ValidateWorkspaceBounds", "default_planning_request_adapters/CheckStartStateBounds", "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization", "default_planning_response_adapters/ValidateSolution"],
            "start_state_max_bounds_error": 0.1,
            "planner_configs": {"RRTConnect": {"type": "geometric::RRTConnect", "range": 0.0}},
            GROUP: {"planner_configs": ["RRTConnect"]},
        },
        "allow_trajectory_execution": False,
    }}}


def export(scene: Scene, out_dir: str) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{scene.robot_name}.urdf").write_text(to_urdf(scene), encoding="utf-8")
    (out / f"{scene.robot_name}.srdf").write_text(to_srdf(scene), encoding="utf-8")
    path = out / "move_group.yaml"
    path.write_text(yaml.safe_dump(move_group_params(scene), sort_keys=False), encoding="utf-8")
    return path


if __name__ == "__main__":
    print(export(Scene.load(sys.argv[1]), sys.argv[2]))
