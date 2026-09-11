# Contributing

Bug reports and focused pull requests are welcome. Open an issue before a large
change so its scope and interface impact can be agreed on first.

## Development

CI builds and tests Humble on Ubuntu 22.04, Jazzy and Kilted on Ubuntu 24.04,
and Lyrical and Rolling on Ubuntu 26.04, on both amd64 and arm64.
Rolling checks forward compatibility; it is not a stable release target.
Use the same `main` branch for all distributions.

Keep C++20 usage compatible with Humble's GCC 11 and libstdc++, and keep CMake
commands compatible with its CMake 3.22. Newer environments must also pass:
Lyrical and Rolling use CMake 4. Prefer CMake imported targets for dependencies.

For example, with ROS 2 Jazzy on Ubuntu 24.04, from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select grasp_synergy_adapter
colcon test --packages-select grasp_synergy_adapter --event-handlers console_direct+
colcon test-result --test-result-base build/grasp_synergy_adapter --verbose
```

Replace `jazzy` in the setup command when testing another distribution.
Inspect each distribution's CI result; a passing Jazzy job does not establish
compatibility with the other distributions.

Keep changes narrowly scoped. New or changed behavior must include a regression
test and corresponding documentation. Run `clang-format` on changed C++ files
and keep the existing CMake formatting.

## Pull requests

- Explain the problem and the behavior changed by the pull request.
- Note any ROS interface, parameter, configuration, or compatibility impact.
- Keep commits reviewable and free of generated build artifacts.
- Resolve test, lint, and documentation-build failures before requesting review.

Contributions are accepted under the Apache License 2.0.
