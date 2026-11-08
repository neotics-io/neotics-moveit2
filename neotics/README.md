# Neotics kit for MoveIt 2

This folder is Neotics' own work, added to our copy of [MoveIt 2](https://github.com/moveit/moveit2). It turns the arm in any Neotics scene into a robot MoveIt can plan for.

It is one part of the Neotics stack. Every fork in [neotics-io](https://github.com/neotics-io) reads the same two files:

- a **Neotics Scene** (`samples/depot.scene.json`): the room, the obstacles, the arm and the named places;
- a **Neotics Episode** (`samples/depot.episode.json`): a run recorded in MuJoCo, with time, joint positions and commands.

The depot is a sample made for these tools. It is not an entry or an answer for any [Neotics challenge](https://lab.neotics.io).

## Use it

Run from this folder:

```bash
python scene_to_moveit.py samples/depot.scene.json out
ros2 run moveit_ros_move_group move_group --ros-args --params-file out/move_group.yaml
```

## How it's checked

Starts MoveIt on ROS 2 Jazzy with the exported robot and plans a real motion from home to reach. This runs on GitHub in `.github/workflows/neotics.yml`, on every change to this folder.

## Licence

See [`LICENSE`](LICENSE). It covers this folder only.
