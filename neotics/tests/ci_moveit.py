"""Full check inside ROS 2: MoveIt loads the exported robot and plans a real motion from home to reach."""
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.node import Node

KIT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(KIT))

from neotics_scene import Scene  # noqa: E402
from scene_to_moveit import GROUP, export  # noqa: E402

SUCCESS = 1


def main() -> int:
    scene = Scene.load(KIT / "samples" / "depot.scene.json")
    params = export(scene, tempfile.mkdtemp())
    move_group = subprocess.Popen(["ros2", "run", "moveit_ros_move_group", "move_group", "--ros-args", "--params-file", str(params)])
    rclpy.init()
    node = Node("neotics_ci")
    client = node.create_client(GetMotionPlan, "plan_kinematic_path")
    try:
        if not client.wait_for_service(timeout_sec=120):
            print("FAIL: move_group never offered planning")
            return 1
        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name, plan.pipeline_id, plan.planner_id = GROUP, "ompl", "RRTConnect"
        plan.allowed_planning_time, plan.num_planning_attempts = 10.0, 1
        plan.max_velocity_scaling_factor, plan.max_acceleration_scaling_factor = 1.0, 1.0
        plan.start_state.joint_state.name = scene.joint_names
        plan.start_state.joint_state.position = [float(q) for q in scene.home]
        plan.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=name, position=float(q), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            for name, q in zip(scene.joint_names, scene.reach)
        ])]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=60)
        response = future.result()
        if response is None:
            print("FAIL: no answer from move_group")
            return 1
        code = response.motion_plan_response.error_code.val
        points = response.motion_plan_response.trajectory.joint_trajectory.points
        print(f"plan result code {code}, {len(points)} points")
        if code != SUCCESS or not points:
            return 1
        missed = [abs(a - b) for a, b in zip(points[-1].positions, scene.reach) if abs(a - b) > 0.02]
        return 1 if missed else 0
    finally:
        move_group.send_signal(signal.SIGINT)
        time.sleep(1)
        move_group.kill()


if __name__ == "__main__":
    sys.exit(main())
