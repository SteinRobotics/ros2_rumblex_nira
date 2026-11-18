# HLS servo implementation

Nira uses Feetech HLS servos. The movement package keeps three responsibilities:

- `CServoController::dashboard` resolves configured names, rejects configured ID
  collisions, updates confirmed IDs, and timestamps complete feedback.
- `CFeetechProtocol` implements robot-unit conversion and HLS operations, including
  the dashboard service in `feetech_dashboard.cpp`. It uses the existing service
  types directly; there is no second command model or conversion layer.
- `FeetechHlsServo` builds the HLS motion payload and owns the register map and
  signed-word encoding. `FeetechPacketProtocol` supplies packet framing and
  `FeetechSerialTransport` supplies serial I/O.
  These two shared layers are required even though SCSCL and SMS/STS model drivers
  have been removed.

Dashboard positions, limits and offsets use native encoder ticks. Ordinary motion
commands retain the controller's legacy 0..1000 position conversion. They share
immediate/deferred motion construction, but dashboard moves intentionally preserve
separate profile/goal writes and readback checks.

Dashboard ID/limit writes do not automatically unlock EEPROM. Offset storage and
calibration disable torque, verify unlocking, attempt the operation, and relock
also after operation failure. Torque stays off. Writes are never retried
implicitly. A failed acknowledgement/readback after a write is `UNVERIFIED`, not
`REJECTED`. Telemetry is assigned only after every register read succeeds. A
confirmed ID change survives subsequent telemetry failure.

`test_feetech_protocol` exercises register transactions and actual HLS packet
construction using in-memory transports. `test_servo_controller` checks ROS service
integration; `test_feetech_serial_transport` checks a pseudo-terminal. These do
not replace a hardware check of EEPROM behavior, motion and calibration on an HLS servo.

## Transport reliability

`FeetechSerialTransport` owns its descriptor, closes it on destruction and
initialization failure, and cannot be copied. Opening a port rejects unsupported baud rates; changing the
baud rate applies the settings to the device. Packet transmission and response
parsing share one monotonic deadline (`io_timeout_ms_`, milliseconds). Partial writes
continue from the unsent suffix; interrupted and temporarily unavailable I/O can
resume within that deadline. A failed transaction is never automatically replayed.

Transmit buffers and packet lengths are bounded. Timeout, serial I/O, checksum,
address, length, and servo-status errors have distinct error codes. Only a complete,
validated, zero-status reply can update read output. A valid servo-error reply
still counts as an occupied address during discovery. Dashboard writes retain
`UNVERIFIED` after an uncertain outcome and include the error reason.

Native position commands require valid single-turn position feedback before
computing a motion profile. Dashboard moves require valid travel limits instead
of substituting full travel. Unused synchronous read/write and reset operations
have been removed from the packet interface.
