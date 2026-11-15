# Split validation — 2026-11-15

All 14 packages compiled and installed locally with tests enabled in the independent
`colcon_nira` workspace. Its Python entry points use `../../.venv/bin/python`.
No hardware deployment, simulation session or CAD export was performed.

## Results

- Registered CTest checks: **56 passed, 2 failed, 58 total** across utilities,
  brain, movement, navigation, description, bringup, Gazebo, camera and face perception.
- Communication/HMI/teleoperation checks in the dedicated `.venv`:
  **92 passed, 1 failed, 1 skipped** (private wake-word model absent).
- All movement, navigation, model, sensor-launch, camera, face-perception and
  setup-script checks passed.
- Python/shell syntax, local package dependencies, CAD imports and documentation
  links checked successfully.
- The six meshes, anatomy configuration and servo calibration match the original
  Nira files byte-for-byte. The repository retains its preparation commit;
  the old repository's history was not imported.

## Existing test mismatches preserved

1. `test_choreography`: the checked-in metal dance includes a music action absent
   from its generator output. The corresponding test also failed in the original repository.
2. `test_single_pose_target` / `FlyingKeepsKneesStraightAndWingsAttachedThroughoutRolls`:
   the test requires 90-degree knees while `flying.yaml` commands 80 and 110 degrees.
   The sequence, test, gait controller and single-pose implementation match the
   original source after package-name substitutions. This existing inconsistency
   was preserved rather than changing robot motion during the split.
3. `test_repository_catalog`: its expected list omits eight existing sequences.
   The corresponding test also failed in the original repository. Catalog code,
   tests and sequences were retained with updated package references.

## Independent Python environment

`../../.venv` contains only the runtime/test requirements declared in `requirements/`
and their dependencies. CAD libraries remain separate. System-site packages are
visible for ROS ABI compatibility; the environment helper disables user-site packages
and removes inherited sibling workspace overlays.

The shell initially inherited the old workspace's paths. The final environment
uses `scripts/environment.sh`, and the remaining packages and Python entry points
were built with clean paths. Generated workspace setup files, CMake caches, runtime
imports and package metadata were checked for old-workspace references. Runtime
imports, including OpenCV/cv_bridge, passed; old package metadata and CAD libraries
were absent from the selected environment.

## Resource controls

Builds, dependency installation and tests ran sequentially with MemoryHigh=6G,
MemoryMax=8G, MemorySwapMax=1G and a 300-second timeout. Compilation used two
workers and tests used one. Build batches completed within these limits.

Private speech models, credentials, system-level hardware drivers and physical
hardware operation are not validated by these software checks. No repositories
were committed, pushed or deployed by this split.
