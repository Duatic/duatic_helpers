^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package duatic_helpers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2026-11-06)
------------------
* fix(startup)!: wait for all controllers to be loaded before querying them (#52)
* deprecate!: remove joint_state_provider_node (#43)
* feat(test_helper): fail wait_for_controller_states on unexpected controllers (#57)
* feat(test_helper): add wait_for_controller_states (#55)
* feat: support the platform velocity controller and the hardware e-stop (#30)
* feat: support ROS 2 lyrical
* feat: C++ nodes in duatic_helpers (#20)
* feat: DynaArm extensions: parameter, Pinocchio and robot structure helpers (#17)
* fix(deps): drop the unused osrf_pycommon (#54)
* fix: use a timer-based wait in the JTC helper (#51)
* fix(joint-states): stop publishing and caching empty joint states (#40)
* fix(robots-helper): strip XML comments before robot structure detection (#35)
* fix: correct the message size check (#34)
