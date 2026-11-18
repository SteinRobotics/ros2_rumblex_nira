/*
 * feetech_hls_servo.hpp
 * Feetech HLS servo operations
 * Date: 2025.9.27
 */

#pragma once

// HLS register addresses use the vendor's HLSCL_ names. A word occupies the
// consecutive _L and _H addresses. These are native device units; robot-angle
// conversion and dashboard validation belong to CFeetechProtocol.
// Register map
//------- EEPROM (read-only) -------
#define HLSCL_MODEL_L 3
#define HLSCL_MODEL_H 4

//------- EEPROM (read/write) -------
#define HLSCL_ID 5
#define HLSCL_BAUD_RATE 6
#define HLSCL_SECOND_ID 7
#define HLSCL_MIN_ANGLE_LIMIT_L 9
#define HLSCL_MIN_ANGLE_LIMIT_H 10
#define HLSCL_MAX_ANGLE_LIMIT_L 11
#define HLSCL_MAX_ANGLE_LIMIT_H 12
#define HLSCL_CW_DEAD 26
#define HLSCL_CCW_DEAD 27
#define HLSCL_OFS_L 31
#define HLSCL_OFS_H 32
#define HLSCL_MODE 33

//------- SRAM (read/write) -------
#define HLSCL_TORQUE_ENABLE 40
#define HLSCL_ACC 41
#define HLSCL_GOAL_POSITION_L 42
#define HLSCL_GOAL_POSITION_H 43
#define HLSCL_GOAL_TORQUE_L 44
#define HLSCL_GOAL_TORQUE_H 45
#define HLSCL_GOAL_SPEED_L 46
#define HLSCL_GOAL_SPEED_H 47
#define HLSCL_TORQUE_LIMIT_L 48
#define HLSCL_TORQUE_LIMIT_H 49
#define HLSCL_LOCK 55

//------- SRAM (read-only) -------
#define HLSCL_PRESENT_POSITION_L 56
#define HLSCL_PRESENT_POSITION_H 57
#define HLSCL_PRESENT_SPEED_L 58
#define HLSCL_PRESENT_SPEED_H 59
#define HLSCL_PRESENT_LOAD_L 60
#define HLSCL_PRESENT_LOAD_H 61
#define HLSCL_PRESENT_VOLTAGE 62
#define HLSCL_PRESENT_TEMPERATURE 63
#define HLSCL_STATUS 65
#define HLSCL_MOVING 66
#define HLSCL_PRESENT_CURRENT_L 69
#define HLSCL_PRESENT_CURRENT_H 70

#include <cstdint>

#include "feetech_serial_transport.hpp"

namespace nira_movement::hls {
// HLS words use sign/magnitude, not two's complement. Decode only successful reads.
constexpr uint16_t encodeSigned(int16_t value) {
    return value < 0 ? static_cast<uint16_t>(-static_cast<int>(value)) | 0x8000 : value;
}
constexpr int decodeSigned(uint16_t value) {
    return value & 0x8000 ? -(value & 0x7fff) : value;
}
}  // namespace nira_movement::hls

// HLS-specific register operations on a Feetech serial bus. This class selects
// registers and builds motion payloads; FeetechPacketProtocol validates replies
// and FeetechSerialTransport owns the port. It performs no ROS work or retries.
// Write methods return 1 on success and 0 on failure. Legacy signed reads return
// -1 on failure; use readWord plus getLastError when -1 is a valid signed value.
class FeetechHlsServo : public FeetechSerialTransport {
   public:
    FeetechHlsServo();
    int WritePosEx(u8 id, s16 position, u16 speed, u8 acc, u16 torque);
    // Stages motion until RegWriteAction is sent; staging alone does not start it.
    int RegWritePosEx(u8 id, s16 position, u16 speed, u8 acc, u16 torque);
    int WriteSpe(u8 id, s16 speed, u8 acc, u16 torque);
    int ServoMode(u8 id);
    int WheelMode(u8 id);
    int EnableTorque(u8 id, u8 enable);
    // Disables torque before unlocking. The caller must subsequently relock,
    // even if a write fails, and must not automatically re-enable torque.
    int unLockEprom(u8 id);
    int LockEprom(u8 id);
    int ReadPos(int id);
    int ReadSpeed(int id);
    int ReadVoltage(int id);
    int ReadTemper(int id);

   private:
    int writeMotion(u8 id, s16 position, u16 speed, u8 acc, u16 torque, bool deferred);
};
