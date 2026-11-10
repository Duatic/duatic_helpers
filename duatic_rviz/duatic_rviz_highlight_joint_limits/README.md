# duatic_rviz_highlight_joint_limits

Highlight `joint_limits` for `duatic_rviz_highlight`: recolors the links behind every joint that is close
to one of its limits: orange (warning) when it is less than the warning margin away from a limit, red (critical) when it
is less than the critical margin away (or beyond the limit). The links are the ones that the joint moves: its child link, and all links
attached to it through fixed joints, up to the next movable joint. Revolute joints that have a lower and an upper limit are monitored, using the
`joint_states` topic.

Use it by adding `joint_limits` to the parameter `highlight.enabled` (a list) of the RViz node, with the
`HighlightedRobotModel` display in the RViz config. Its parameters (of the RViz node, so they can be set from a launch
file), all under `highlight.joint_limits.`:

| Parameter | Default | |
| --- | --- | --- |
| `mode` | `absolute_deg` | `absolute_deg`: margins in degrees. `relative`: margins as a fraction of the joint's range. |
| `margin.absolute_deg.warning`, `margin.absolute_deg.critical` | 15, 2.5 | margins in degrees, in the mode `absolute_deg` |
| `margin.relative.warning`, `margin.relative.critical` | 0.06, 0.01 | margins as a fraction of the range (0 to 0.5), in the mode `relative` |

For example, `{"joint_limits": {"mode": "relative", "margin": {"relative": {"warning": 0.15}}}}` colors a joint orange
when less than 15 % (0.15) of its range is left. Only the margins of the selected mode are used. Values are validated: an unknown mode or a
negative margin is rejected when the parameter is set. The parameters can be changed while RViz runs.
