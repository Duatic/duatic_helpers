# duatic_bringup_stages

Starts a robot bringup in stages. A stage is a group of launch actions plus a gate, a process that exits 0 once the stage is up. A stage starts when every stage it comes `after` is ready. If a required stage does not come up, the launch shuts down and names what failed.

```python
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from duatic_bringup_stages.stages import Stage, gate, stage_arguments, staged

stages = [
    Stage("imu", [imu_driver], gate("imu", 30, topics=["sensor/imu/data"])),
    Stage("lidars", [lidar_driver], gate("lidars", 30, topics=["sensor/lidar/scan"])),
    Stage(
        "state_estimation",
        [ekf],
        gate("state_estimation", 30, topics=["odometry/filtered"]),
        after=["imu"],
    ),
]
return [
    *stage_arguments(["imu", "lidars", "state_estimation"]),
    GroupAction([PushRosNamespace("robot1"), staged(stages)]),
]
```

Each robot bringup package defines its stages in its own `stages.py`, for example `duatic_duarover_bringup.stages`.

At startup the launch logs the graph, for example `Stages: core; imu; state_estimation after core, imu; navigation after state_estimation (optional)`.

## Gates
`gate(name, timeout, ...)` checks any combination of:

- `topics`: a first message on each topic
- `controllers_config`: every controller in a ros2_control parameters file is in its configured state, `inactive` if its `state` says so, otherwise `active`. The file needs the `/**: controller_manager: ros__parameters:` layout.
- `hardware_components_active`: every hardware component of the controller manager is active
- `lifecycle_nodes`: each node is in the active state

`controller_manager` names the controller manager for the controller and hardware checks. The default, `controller_manager`, resolves in the stage's namespace.

After `timeout` seconds the gate logs every check that has not passed and exits 1. Timeouts run on wall time, also under `use_sim_time`. The `stage_timeout_scale` argument multiplies every gate timeout, for example `stage_timeout_scale:=5` for a slow simulator start or a simulator that runs slower than real time.

A gate only checks the start of its stage. Nothing watches a stage once it is ready.

## Stages
- Every configurable stage has a `require_<stage>` argument, `true` by default. If a required stage fails, the launch shuts down with the reason and `ros2 launch` exits with code 1. If it is set to `false`, the launch stops the processes of the stage, continues without it and skips the stages that come after it. A skipped stage that is itself required still shuts the launch down. A stage built with `configurable=False` is always required.
- A failed optional stage stops the processes it started, also those in launch files it includes. A process its actions start only later, for example from an event handler, is not stopped. A process that shuts the launch down when it exits, through `on_exit`, does so also when its stage is stopped.
- `abort(reason)` returns an action that shuts the launch down so that `ros2 launch` exits with code 1, for example as the `on_exit` of a process the launch cannot run without.
- Declare the `require_<stage>` and `stage_timeout_scale` arguments with `stage_arguments` in `generate_launch_description`, so that `ros2 launch <package> <file> --show-args` lists them.
- Place `staged(...)` where the stages should take their namespace, launch arguments and environment from, for example after `PushRosNamespace` in a `GroupAction`. Each stage runs in its own scope, so launch configurations set by one stage are not seen by the others.
- Run one `staged(...)` per robot. A robot built from other robots puts their stages into its own graph, so that one call lists every stage and what it waits for. A launch file started by a stage can still run its own `staged(...)`, but those stages are not part of the outer graph: stages that come after the outer stage do not wait for them. Stage names must be unique across all graphs in one namespace, because each gate node is named `<stage>_gate`.
- Stage names may contain letters, digits and underscores, and start with a letter.
