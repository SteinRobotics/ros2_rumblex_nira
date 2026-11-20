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

## Standing and walking pose overlays

Tap **A / Cross** to toggle Follow Face and **B / Circle** to toggle Level Torso.
Both start off and can run together. They apply during stand-up, standing hold,
and all five move gaits. Other gaits pause the features. Returning to stand-up or
walking resumes them with fresh sensor data.

Face following uses up to 5° of torso yaw and pitch before directing the head,
and up to 5° of torso roll for eye-line tilt. The fixed desktop camera supplies
an absolute pointing direction. The calibrated head camera supplies pointing
error relative to its current view. IMU leveling adds a separate roll/pitch
correction. The final orientation respects remaining travel around the gait pose.
During move gaits, Follow Face replaces the gait’s side-to-side head scan with
tracking yaw. The scan resumes when Follow Face is switched off.
Switching a feature off or losing its input slews its correction back to zero.

Face tracking settings live in `config/nira/parameter.yaml` and leveling settings
in `config/nira/gait.yaml`. See [controller and hardware commands](../COMMANDS.md).
