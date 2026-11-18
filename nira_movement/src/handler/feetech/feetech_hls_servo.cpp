#include "feetech_hls_servo.hpp"

using nira_movement::hls::decodeSigned;
using nira_movement::hls::encodeSigned;

FeetechHlsServo::FeetechHlsServo() {
    // HLS register words are transmitted low byte first.
    byte_order_ = 0;
}

int FeetechHlsServo::writeMotion(u8 id, s16 position, u16 speed, u8 acc, u16 torque, bool deferred) {
    // Registers 41..47 form one contiguous motion command: acceleration,
    // signed-magnitude position, torque, then speed (all words little-endian).
    // Both immediate and deferred commands must have exactly this payload.
    u8 data[7] = {acc};
    Host2SCS(data + 1, data + 2, encodeSigned(position));
    Host2SCS(data + 3, data + 4, torque);
    Host2SCS(data + 5, data + 6, speed);
    return deferred ? regWrite(id, HLSCL_ACC, data, sizeof(data))
                    : genWrite(id, HLSCL_ACC, data, sizeof(data));
}
int FeetechHlsServo::WritePosEx(u8 id, s16 position, u16 speed, u8 acc, u16 torque) {
    return writeMotion(id, position, speed, acc, torque, false);
}
int FeetechHlsServo::RegWritePosEx(u8 id, s16 position, u16 speed, u8 acc, u16 torque) {
    return writeMotion(id, position, speed, acc, torque, true);
}
int FeetechHlsServo::WriteSpe(u8 id, s16 speed, u8 acc, u16 torque) {
    // Wheel mode uses a signed speed and a zero position field. Position mode
    // instead uses a nonnegative speed magnitude in the same register pair.
    return writeMotion(id, 0, encodeSigned(speed), acc, torque, false);
}
int FeetechHlsServo::ServoMode(u8 id) {
    return writeByte(id, HLSCL_MODE, 0);
}
int FeetechHlsServo::WheelMode(u8 id) {
    return writeByte(id, HLSCL_MODE, 1);
}
int FeetechHlsServo::EnableTorque(u8 id, u8 enable) {
    return writeByte(id, HLSCL_TORQUE_ENABLE, enable);
}
int FeetechHlsServo::unLockEprom(u8 id) {
    if (EnableTorque(id, 0) != 1) return 0;
    return writeByte(id, HLSCL_LOCK, 0);
}
int FeetechHlsServo::LockEprom(u8 id) {
    return writeByte(id, HLSCL_LOCK, 1);
}
int FeetechHlsServo::ReadPos(int id) {
    // Check transport failure before decoding: interpreting -1 as a register
    // word would manufacture a large negative position from a failed read.
    const int kValue = readWord(id, HLSCL_PRESENT_POSITION_L);
    return kValue < 0 ? -1 : decodeSigned(kValue);
}
int FeetechHlsServo::ReadSpeed(int id) {
    const int kValue = readWord(id, HLSCL_PRESENT_SPEED_L);
    return kValue < 0 ? -1 : decodeSigned(kValue);
}
int FeetechHlsServo::ReadVoltage(int id) {
    return readByte(id, HLSCL_PRESENT_VOLTAGE);
}
int FeetechHlsServo::ReadTemper(int id) {
    return readByte(id, HLSCL_PRESENT_TEMPERATURE);
}
