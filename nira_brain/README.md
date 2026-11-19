# brain

- requests navigation goals
- contains system checks

## Physical quantities

Brain uses the shared mp-units types from `nira_utils/units.hpp` for lengths,
angles, durations, linear and angular velocities, voltages, and temperatures.
Gait `start`/`updateTimed` methods and movement request durations require typed
seconds, for example `gait.start(1.0 * s, direction)`. Use short mp-units names with
`using namespace mp_units;` and `using namespace mp_units::si::unit_symbols;`.
Explicitly import angular `rad`/`deg` symbols to retain the distinct angle dimension.
Continuous gaits
accept `brain::Velocity`, whose linear components are in m/s and angular
components are in rad/s. Angles retain a distinct dimension, so angular velocity
cannot be substituted for linear velocity.

ROS messages and YAML retain their existing numeric formats: metres, seconds,
degrees for poses, radians/second for Twist angular velocities, volts, and Celsius.
`Velocity::fromMsg`/`toMsg` handle the velocity boundary; `limitVelocity` validates
all six ROS components before producing a limited planar velocity. ROS and
steady-clock timestamps remain clock types, with durations converted at the
clock boundary.

Temperature readings and thresholds are Celsius points, created with
`units::celsius(value)` and converted for display with `units::inCelsius(value)`.
Filtering adds a fraction of the sample-minus-previous temperature difference to
the previous reading. The initial filtered temperature remains 0°C; temperature
errors use the raw sample, while voltage errors use filtered voltage.

Normalized trajectory progress, filter coefficients, joystick deflection, gait
selection thresholds remain scalars. Gait timing,
velocity limits, robot-specific configuration, and monitoring thresholds are
unchanged. See [MOVEMENT.md](MOVEMENT.md) for the motion pipeline and build setup.

## Offline face preview

Nira's desktop test launch enables `face_tracking.desktop_preview` in the brain.
For the fixed host camera, the torso takes the first 10° of yaw and pitch
independently, and the head takes the remaining angle within its configured
limits. Torso roll mirrors the tracked face’s eye-line tilt up to 10°.
`CMovement` publishes these orientations together while idle, preserving torso
translation and toe positions. Missing observations hold the pose. If only
the eye-line measurement is unavailable, yaw and pitch continue tracking.
Any movement request, nonzero velocity request, or cancellation disables preview;
`/brain/set_face_tracking` explicitly re-enables it once the operator is ready.

Limits/gain are in `config/nira/parameter.yaml`. The mode defaults to false in
`brain_launch.py` and is not wired into physical `target_launch.py`. Physical
camera TF, feedback-based control, actuator limit enforcement, and commissioning
remain required before hardware face tracking is enabled.
