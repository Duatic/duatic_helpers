# duatic_bringup_supervisor

Starts a robot stage by stage from a YAML stage graph, watches it while it runs, and stops it in
reverse order. The launch file only starts processes, all at once: lifecycle nodes come up
unconfigured and do nothing until the supervisor configures and activates them.

## Stage files

```yaml
stages:
  core:
    timeout: 120                     # seconds until the stage must be up
    delegate:
      ros2_control:                  # controllers and hardware, as one unit
        controllers_config: ${controllers_config}
  imu:
    timeout: 30
    lifecycle_nodes: [imu_driver]    # configured and activated in this order
  lidars:
    timeout: 30
    ready:
      topics:                        # for parts that are plain nodes
        - {name: sensor/lidar/scan, max_age: 1.0}
  navigation:
    after: [core, imu, lidars]       # starts once these are ready
    required: false                  # a failure stops this stage, not the robot
    timeout: 150
    delegate:
      nav2: [lifecycle_manager_navigation]
```

| Key | Meaning |
| --- | --- |
| `after` | Stages that must be ready before this one starts. Stages without a path between them start in parallel. |
| `required` | Default `true`. A failed required stage stops the robot and the supervisor exits with code 1. A failed optional stage is stopped with every stage that needs it. |
| `timeout` | Seconds from the start of the stage until all its parts are up. |
| `lifecycle_nodes` | Lifecycle nodes the supervisor configures and activates in order. |
| `delegate.nav2` | Nav2 lifecycle managers, started with `manage_nodes` STARTUP and watched with `is_active`. Start them with `autostart:=false`. |
| `delegate.ros2_control` | A controller manager. The manager activates its hardware and the spawners the controllers; the stage is ready once every controller in `controllers_config` is in its configured state and all hardware is active. On stop the supervisor deactivates the controllers, then the hardware. `manage: false` only watches it, for a manager that another bringup owns. |
| `ready.topics` | Topics that must publish. `max_age` also fails the stage when the topic goes quiet after startup. |

Several files are merged in order: the base file first, then overlays. Mappings merge key by key,
lists are replaced, and `null` removes a key, so an overlay can drop a whole stage. `${name}` takes
the value of the supervisor's `vars.<name>` parameter.

## Launch

```python
from duatic_bringup_supervisor.launch_actions import stage_arguments, supervisor

# in generate_launch_description: require_<stage> and stage_timeout_scale
*stage_arguments(stages_directory),
# next to the processes, in the robot's namespace
supervisor([base, overlay], variables={"controllers_config": path}, optional_files=[robot]),
```

`require_<stage>:=true|false` overrides a stage's `required`; empty keeps what the files say.
`stage_timeout_scale` multiplies every timeout. Files in `optional_files` apply last and only when
they exist, for a robot's own configuration. When the supervisor exits, the launch ends; a
non-zero exit makes `ros2 launch` fail.

## Running

- Every stage's state is published on `diagnostics`, one status per stage.
- `ros2 service call <ns>/bringup_supervisor/shutdown std_srvs/srv/Trigger` stops the stages in
  reverse dependency order (Nav2 before state estimation before controllers and hardware), then
  the supervisor exits and the launch ends with it.
- Ctrl-C on `ros2 launch` signals every process at once, so it gives no ordered stop.
- The supervisor cannot stop plain nodes: when a stage with plain nodes fails, those keep running.
- Safety functions (e-stop, brakes) must not depend on the supervisor.
