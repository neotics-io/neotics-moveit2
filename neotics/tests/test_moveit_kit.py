import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import pytest
import yaml

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_moveit import GROUP, export, to_srdf  # noqa: E402


@pytest.fixture
def scene():
    return Scene.load(KIT / "samples" / "depot.scene.json")


def test_srdf_defines_the_group_and_both_poses(scene):
    robot = ET.fromstring(to_srdf(scene))
    assert robot.find("group").get("name") == GROUP
    states = {s.get("name"): [float(j.get("value")) for j in s.findall("joint")] for s in robot.findall("group_state")}
    assert states == {"home": scene.home, "reach": scene.reach}


def test_only_neighbouring_links_skip_collision_checks(scene):
    pairs = ET.fromstring(to_srdf(scene)).findall("disable_collisions")
    assert len(pairs) == len(scene.links) - 1 and all(p.get("reason") == "Adjacent" for p in pairs)


def test_parameter_file_is_complete(scene, tmp_path):
    params = yaml.safe_load(export(scene, str(tmp_path)).read_text(encoding="utf-8"))["move_group"]["ros__parameters"]
    assert "<robot" in params["robot_description"] and "<group" in params["robot_description_semantic"]
    assert list(params["robot_description_planning"]["joint_limits"]) == scene.joint_names
    assert (tmp_path / "neotics_arm.urdf").exists() and (tmp_path / "neotics_arm.srdf").exists()
