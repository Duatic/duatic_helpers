# duatic_rviz_highlight

RViz plugin that recolors the links of the robot model while RViz runs, e.g. to show which joints are
close to a limit. This package provides **no highlights itself**; those live in the packages that
implement them, e.g. `duatic_rviz_highlight_joint_limits`.

RViz has no interface to change the look of the `RobotModel` display from outside. The display
`duatic_rviz_highlight/HighlightedRobotModel` is a `RobotModel` display (same properties, same behavior)
that additionally runs highlights, and recolors the links they ask for. A link that no highlight asks for
keeps its own look.

## Use

In the RViz config, use the display instead of `rviz_default_plugins/RobotModel`:

```yaml
- Class: duatic_rviz_highlight/HighlightedRobotModel
  Name: RobotModel
```

The highlights are chosen by the string array parameter `highlight.enabled` of the RViz node (none by
default), e.g. in a launch file:

```python
Node(
    package="rviz2",
    executable="rviz2",
    namespace=namespace,
    arguments=["-d", rviz_config],
    parameters=[
        {
            "highlight": {
                "enabled": ["<highlight-plugin>"],
                "<highlight-plugin>": {"<param1>": <value1>, ...},  # parameters of a highlight
            }
        }
    ],
    remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
)
```

All of them are in the section `highlight` of the parameters of the RViz node: `highlight.enabled` selects the
highlights, and the parameters of a highlight are named `highlight.<highlight name>.<parameter>` (so a highlight
cannot be called `enabled`). On the command line, a list is written in yaml syntax:

```bash
ros2 run rviz2 rviz2 -d config.rviz --ros-args -p highlight.enabled:="[joint_limits, self_collision]"
```

A parameter cannot be an empty list: RViz does not start with `-p highlight.enabled:="[]"`. Use `[""]`
(a list with an empty name) for no highlights, empty names are ignored.

The topics of a highlight (e.g. `joint_states`) are relative to the namespace of the RViz node.

If several highlights color the same link, the one with the higher priority wins. The priority is the parameter
`highlight.<highlight name>.priority` (each highlight has its own default, and it can be changed while RViz runs). With
equal priorities, the later highlight in `highlight.enabled` wins.

A highlight that throws in `update` or `onRobotDescription` is logged and unloaded, until `highlight.enabled` changes.

Both description sources of the `RobotModel` display work (topic and file). Highlights are told about the robot
description every time one is loaded.

## Provide a highlight

A highlight is a plugin of this package, so it can live in a separate (e.g. private) package that depends on
`duatic_rviz_highlight`. Derive from `duatic::rviz_highlight::HighlightPlugin<ParamListener>`
(`plugin/highlight_plugin.hpp`). The parameters of the highlight are declared by a
[generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) `ParamListener`, which is
created with the parameter prefix `highlight.<name>.` and read from `params_` (the display refreshes it once per frame). They must include an int
`priority`: the default priority of the highlight.

```cpp
class MyHighlight : public duatic::rviz_highlight::HighlightPlugin<my_highlight::ParamListener>
{
public:
  void onRobotDescription(const std::string& urdf_xml) override;  // optional
  void update(duatic::rviz_highlight::LinkColors& colors) override;  // every frame

protected:
  void onInitialize() override;  // optional: create subscriptions etc.
  void onParametersChanged() override;  // optional: after initialization and whenever the parameters changed
};

DUATIC_RVIZ_HIGHLIGHT_EXPORT_CLASS(MyHighlight)
```

In `update`, set a color per link, e.g. `colors["link"] = duatic::rviz_highlight::palette::kRed`, or your own with
`Color::rgb(255, 140, 0)` (`common/color.hpp`). List the plugin in a plugin description file (the `name` is what is given as
the highlight name):

```xml
<library path="my_package">
  <class name="my_highlight" type="MyHighlight" base_class_type="duatic::rviz_highlight::HighlightPlugin"/>
</library>
```

and export it in `CMakeLists.txt`:

```cmake
pluginlib_export_plugin_description_file(duatic_rviz_highlight plugins_description.xml)
```

Everything of a highlight runs on the RViz thread: RViz spins its node (and so the callbacks of the highlight's
subscriptions) in the same timer that calls `update`. So no locking is needed, unless the highlight creates its own
threads or executors.
