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

A robot keeps its stage files in `<package share>/config/stages/`: `stages.yaml` is the base,
every other file is an overlay named after what selects it (`sim`, `mock`, `mapping`, ...). Files
merge in order: mappings key by key, lists are replaced, and `null` removes a key, so an overlay
can drop a whole stage. `${name}` takes the value of the supervisor's `vars.<name>` parameter.

## Robots built from robots

One supervisor runs the whole robot. A robot built from other robots includes their stage files
instead of starting their supervisors:

```yaml
# duatic_dxtr_bringup/config/stages/stages.yaml
include:
  rover:
    package: duatic_duarover_bringup
    bind: {core: core}                 # the rover's core is the DXTR's core
  torso:
    package: duatic_duatorso_bringup
    bind: {core: core}
stages:
  core: {timeout: 120, delegate: {ros2_control: {controllers_config: ${controllers_config}}}}
```

| Include key | Meaning |
| --- | --- |
| prefix (the key) | Included stage `imu` becomes `rover_imu`, also in `after` lists and `require_<stage>` |
| `package` | The included robot's bringup package, holding `config/stages/` |
| `bind` | Maps an included stage onto a stage of the including robot, which replaces it; used for the one ros2_control a composed robot has |

An included robot runs with the overlays the including robot runs with, those it has, so a DXTR
in simulation runs the rover's `sim.yaml`. Included names must not collide, and every `bind`
target must be a stage of the including robot.

Includes nest (the DXTR includes the torso, which includes both arms) and the including robot's
own `stages` merge last, so it can change any included stage, for example
`rover_navigation: {required: false}`.

## Launch

```python
from duatic_bringup_supervisor.launch_actions import stage_arguments, supervisor

# in generate_launch_description: require_<stage> and stage_timeout_scale
*stage_arguments("my_robot_bringup"),
# in launch_setup(context), next to the processes, in the robot's namespace
*supervisor(context, "my_robot_bringup", ["sim"], variables={"controllers_config": path}),
```

`supervisor()` returns nothing when `start_as_subcomponent` is true, read before any include of
the calling launch file can change it: the robot that includes this one runs its stages.
Overlay names without a file are skipped, so a launch file can pass its variant and map mode as
they are. `require_<stage>:=true|false` overrides a stage's `required`; empty keeps what the
files say, and a set one that names no stage makes the supervisor (and so the launch) fail.
`stage_timeout_scale` multiplies every timeout. `/ros2_ws/robot_config/stages.yaml` applies
last when it exists, for a robot's own configuration. When the supervisor exits, the launch
ends; a non-zero exit makes `ros2 launch` fail.

`duatic_bringup_supervisor.testing.wait_for_stages` waits in a launch test until every stage is
ready.

## Running

- Every stage's state is published on `diagnostics`, one status per stage.
- `ros2 service call <ns>/bringup_supervisor/shutdown std_srvs/srv/Trigger` stops the stages in
  reverse dependency order (Nav2 before state estimation before controllers and hardware), then
  the supervisor exits and the launch ends with it.
- Ctrl-C on `ros2 launch` signals every process at once, so it gives no ordered stop.
- The supervisor cannot stop plain nodes: when a stage with plain nodes fails, those keep running.
- Safety functions (e-stop, brakes) must not depend on the supervisor.
