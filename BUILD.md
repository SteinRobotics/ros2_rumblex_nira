# Build and test Nira

## Terminal environment

```bash
cd ~/Workspace/colcon_nira/src/ros2_rumblex_nira
source scripts/environment.sh
cd ~/Workspace/colcon_nira
```

This selects `/opt/ros/lyrical`, this workspace's `.venv`, and its own installed
overlay. It removes inherited sibling workspace paths and disables user-site Python
packages. System packages remain visible for ROS ABI compatibility. Run
`bash scripts/setup_venv.sh` from the repository first to create the environment;
CAD dependencies stay in the dedicated CAD environment.

Do not reuse build/install directories from the original repository: package names changed.

## Resource limits

Follow [nira_doc/BUILD_AND_TEST.md](nira_doc/BUILD_AND_TEST.md) to define and verify
`nira_limited`. Check existing jobs first. On this amd64 host use MemoryHigh=6G,
MemoryMax=8G, MemorySwapMax=1G and a five-minute timeout, two compiler workers,
sequential packages and one test worker. On Pi use the guide's RAM-relative limits
and one worker. Run heavy workloads sequentially across sessions. After a timeout or memory failure,
inspect logs and resume only a smaller focused batch; never rerun the full workload automatically.

## Build

From the repository root, choose the build for the target machine. Both entry points use the same resource limits as the editor tasks.

```bash
./build_pc.sh                    # all packages in workspace/src
./build_robot.sh                 # physical bringup and its dependencies
./build_robot.sh nira_movement   # focused build within that set
```

For editor tasks and direct calls to `scripts/build.sh`, the wrapper reads the command-line profile first, then `NIRA_BUILD_PROFILE`, then
`${XDG_CONFIG_HOME:-$HOME/.config}/nira/build-profile`. The default is `desktop`.
Pi provisioning writes `robot` to that file. For an existing robot, set
`export NIRA_BUILD_PROFILE=robot` in its build environment or create the same
one-line file. Raw `colcon build` uses colcon's own selection rules.

The robot profile builds Release with tests disabled. Pi provisioning passes the
same source package paths to rosdep, including the lidar driver and any source
checkout of usb_cam. On the desktop use the whole workspace's `src` as the
rosdep input. Follow [setup](SETUP.md) for external drivers and Python dependencies.
To inspect the robot selection from the workspace root after sourcing the environment:

```bash
python -m colcon list --base-paths src --packages-up-to nira_bringup --names-only
```

Switching profiles in an existing workspace leaves previously installed packages
and system dependencies in place. Use a fresh workspace to verify a headless installation.

## Tests

```bash
nira_limited colcon test --executor sequential --packages-select nira_movement \
  --event-handlers console_direct+ --ctest-args --output-on-failure -j1
colcon test-result --verbose
```

Run package tests appropriate to the change. CAD tests use a separate CAD environment
and bounded processes; they are not part of the ordinary software test run.
The section-parser Python script is a CTest harness requiring a compiled executable,
not a pytest collection target. See package CMakeLists.txt for its invocation.
