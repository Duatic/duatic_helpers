# duatic_helpers
[![Jazzy](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/88e2c55697b0173e5904bc7bc224a8fd/raw/duatic_helpers-jazzy.json)](https://github.com/Duatic/duatic_helpers/actions/workflows/ci.yml)
[![Kilted](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/88e2c55697b0173e5904bc7bc224a8fd/raw/duatic_helpers-kilted.json)](https://github.com/Duatic/duatic_helpers/actions/workflows/ci.yml)
[![Lyrical](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/88e2c55697b0173e5904bc7bc224a8fd/raw/duatic_helpers-lyrical.json)](https://github.com/Duatic/duatic_helpers/actions/workflows/ci.yml)
[![Rolling](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/mbloechli/88e2c55697b0173e5904bc7bc224a8fd/raw/duatic_helpers-rolling.json)](https://github.com/Duatic/duatic_helpers/actions/workflows/ci.yml)

This repository contains ROS 2 related helper scripts and robotics libraries used at [Duatic](https://duatic.com).

## Packages

This repository is structured as a monorepo and contains the following packages:

* **`duatic_helpers`**: General ROS 2 support functionality, including launch and testing utilities.
* **`duatic_kinematics`**: High-performance, ROS-agnostic robotics mathematics and kinematics solvers (e.g., JAX-optimized Inverse Kinematics using PyRoki).
* **`duatic_monitoring`**: Hardware monitoring nodes for battery state publishing (gamepad, LiTime BMS, aggregator).

## Installation & Dependencies

While standard ROS dependencies are resolved via `package.xml` and `rosdep`, the `duatic_kinematics` package relies on specific Python libraries (such as custom GitHub forks) that `rosdep` cannot resolve automatically.

Before building your workspace, please ensure you install the Python requirements manually (or add this step to your Dockerfile):

```bash
# Navigate to your workspace root, then run:
pip3 install -r src/duatic_helpers/duatic_kinematics/requirements.txt
```

## License

The contents are licensed under the BSD-3-Clause [license](LICENSE), note that some files are under a different license (Apache 2.0).
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository. Please open an issue in order to get in touch with us.

## Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)
