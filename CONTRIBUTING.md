# Contributing

Bug reports and focused pull requests are welcome. Open an issue before a large
change so its scope and interface impact can be agreed on first.

## Development

Use ROS 2 Jazzy on Ubuntu 24.04. From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select grasp_synergy_adapter
colcon test --packages-select grasp_synergy_adapter --event-handlers console_direct+
colcon test-result --test-result-base build/grasp_synergy_adapter --verbose
```

Keep changes narrowly scoped. New or changed behavior must include a regression
test and corresponding documentation. Run `clang-format` on changed C++ files
and keep the existing CMake formatting.

## Pull requests

- Explain the problem and the behavior changed by the pull request.
- Note any ROS interface, parameter, configuration, or compatibility impact.
- Keep commits reviewable and free of generated build artifacts.
- Resolve test, lint, and documentation-build failures before requesting review.

Contributions are accepted under the Apache License 2.0.
