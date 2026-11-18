/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 ******************************************************************************/

#include "handler/feetech_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace nira_movement {

CFeetechProtocol::CFeetechProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& kDeviceName)
    : CServoProtocol(node), device_name_(kDeviceName) {
}

CFeetechProtocol::~CFeetechProtocol() {
    closeConnection();
}

int CFeetechProtocol::dashboardRead(uint8_t id, uint8_t address, bool word) {
    if (!is_connected_) return -1;
    return word ? hls_servo_.readWord(id, address) : hls_servo_.readByte(id, address);
}

bool CFeetechProtocol::dashboardWrite(uint8_t id, uint8_t address, uint16_t value, bool word) {
    if (!is_connected_) return false;
    // Deliberately do not unlock EEPROM or retry a write.
    return (word ? hls_servo_.writeWord(id, address, value) : hls_servo_.writeByte(id, address, value)) == 1;
}

bool CFeetechProtocol::triggerConnection() {
    is_connected_ = false;
    RCLCPP_INFO(node_->get_logger(),
                "CFeetechProtocol: Connecting to serial port %s with 115200 baud Rate...",
                device_name_.c_str());
    if (hls_servo_.begin(115200, device_name_.c_str())) {
        is_connected_ = true;
        RCLCPP_INFO(node_->get_logger(), "CFeetechProtocol: Connection successful");
        return true;
    }
    RCLCPP_ERROR(node_->get_logger(), "CFeetechProtocol: Connection failed on port %s", device_name_.c_str());
    return false;
}

void CFeetechProtocol::closeConnection() {
    if (is_connected_) {
        hls_servo_.end();
        is_connected_ = false;
    }
}

bool CFeetechProtocol::isConnected() {
    return is_connected_;
}

bool CFeetechProtocol::getServoID(uint8_t id, uint8_t& answer) {
    if (!is_connected_) return false;
    int res = hls_servo_.Ping(id);
    if (res != -1 || hls_servo_.getLastError() == ERR_SERVO) {
        // A valid error-status response still proves that this address is occupied.
        answer = id;
        return true;
    }
    return false;
}

bool CFeetechProtocol::setServoID(uint8_t id, uint8_t new_id) {
    if (!is_connected_) return false;
    if (hls_servo_.unLockEprom(id) != 1) {
        hls_servo_.LockEprom(id);
        return false;
    }
    int res = hls_servo_.writeByte(id, HLSCL_ID, new_id);
    const int kLocked = hls_servo_.LockEprom(new_id);
    return res == 1 && kLocked == 1;
}

bool CFeetechProtocol::getMode(uint8_t id, uint8_t& mode) {
    if (!is_connected_) return false;
    int res = hls_servo_.readByte(id, HLSCL_MODE);
    if (res != -1) {
        mode = static_cast<uint8_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::setServoMode(uint8_t id) {
    if (!is_connected_) return false;
    return hls_servo_.ServoMode(id) == 1;
}

bool CFeetechProtocol::setMotorMode(uint8_t id, int16_t speed) {
    if (!is_connected_) return false;
    if (hls_servo_.WheelMode(id) != 1) return false;
    return hls_servo_.WriteSpe(id, speed, 50, 500) == 1;
}

bool CFeetechProtocol::setTorque(uint8_t id, bool active) {
    if (!is_connected_) return false;
    return hls_servo_.EnableTorque(id, active ? 1 : 0) == 1;
}

bool CFeetechProtocol::setPosition(uint8_t id, uint16_t pos, uint16_t time_ms) {
    return writePosition(id, pos, time_ms, false);
}
bool CFeetechProtocol::setRegPos(uint8_t id, uint16_t pos, uint16_t time_ms) {
    return writePosition(id, pos, time_ms, true);
}

bool CFeetechProtocol::writePosition(uint8_t id, uint16_t pos, uint16_t time_ms, bool deferred) {
    if (!isConnected() || id > 253) return false;
    const int kCurrentPos = dashboardRead(id, HLSCL_PRESENT_POSITION_L, true);
    if (kCurrentPos < 0 || kCurrentPos > 4095) return false;
    uint16_t target_pos = convertTicksToFeetech(pos);
    uint16_t delta = std::abs(kCurrentPos - static_cast<int>(target_pos));

    uint16_t speed = 0;
    if (time_ms > 0 && delta > 0) {
        speed = static_cast<uint16_t>(std::clamp(std::round((delta * 20.0) / time_ms), 1.0, 250.0));
    } else {
        speed = 50;  // default safe speed
    }

    return (deferred ? hls_servo_.RegWritePosEx(id, static_cast<s16>(target_pos), speed, 50, 500)
                     : hls_servo_.WritePosEx(id, static_cast<s16>(target_pos), speed, 50, 500)) == 1;
}

bool CFeetechProtocol::actionStart(uint8_t id) {
    if (!is_connected_) return false;
    return hls_servo_.RegWriteAction(id) == 1;
}

bool CFeetechProtocol::moveStop(uint8_t id) {
    if (!is_connected_) return false;
    return hls_servo_.WriteSpe(id, 0, 50, 500) == 1;
}

bool CFeetechProtocol::getPosition(uint8_t id, int16_t& pos) {
    if (!isConnected() || id > 253) return false;
    int res = dashboardRead(id, HLSCL_PRESENT_POSITION_L, true);
    if (res >= 0 && res <= 4095) {
        pos = static_cast<int16_t>(convertFeetechToTicks(static_cast<uint16_t>(res)));
        return true;
    }
    return false;
}

bool CFeetechProtocol::getPositionLimits(uint8_t id, uint16_t& min_position, uint16_t& max_position) {
    if (!is_connected_) return false;
    int min_pos = hls_servo_.readWord(id, HLSCL_MIN_ANGLE_LIMIT_L);
    int max_pos = hls_servo_.readWord(id, HLSCL_MAX_ANGLE_LIMIT_L);
    if (min_pos != -1 && max_pos != -1) {
        min_position = convertFeetechToTicks(static_cast<uint16_t>(min_pos));
        max_position = convertFeetechToTicks(static_cast<uint16_t>(max_pos));
        return true;
    }
    return false;
}

bool CFeetechProtocol::setPositionLimits(uint8_t id, uint16_t min_position, uint16_t max_position) {
    if (!is_connected_) return false;
    uint16_t min_counts = convertTicksToFeetech(min_position);
    uint16_t max_counts = convertTicksToFeetech(max_position);
    if (hls_servo_.unLockEprom(id) != 1) {
        hls_servo_.LockEprom(id);
        return false;
    }
    int r1 = hls_servo_.writeWord(id, HLSCL_MIN_ANGLE_LIMIT_L, min_counts);
    int r2 = hls_servo_.writeWord(id, HLSCL_MAX_ANGLE_LIMIT_L, max_counts);
    const int kLocked = hls_servo_.LockEprom(id);
    return r1 == 1 && r2 == 1 && kLocked == 1;
}

bool CFeetechProtocol::readHardwareOffset(uint8_t id, int16_t& ticks) {
    int raw = dashboardRead(id, HLSCL_OFS_L, true);
    if (raw < 0 || (raw & 0x7fff) > 4095) return false;
    ticks = hls::decodeSigned(raw);
    return true;
}

bool CFeetechProtocol::writeHardwareOffset(uint8_t id, int16_t ticks) {
    if (!isConnected() || id > 253 || ticks < -4095 || ticks > 4095) return false;
    // Never re-enable torque against a target expressed in the old coordinate frame.
    if (!dashboardWrite(id, HLSCL_TORQUE_ENABLE, 0, false) ||
        dashboardRead(id, HLSCL_TORQUE_ENABLE, false) != 0)
        return false;
    bool unlocked = dashboardWrite(id, HLSCL_LOCK, 0, false) && dashboardRead(id, HLSCL_LOCK, false) == 0;
    bool written = unlocked && dashboardWrite(id, HLSCL_OFS_L, hls::encodeSigned(ticks), true);
    bool locked = dashboardWrite(id, HLSCL_LOCK, 1, false);
    int16_t actual = 0;
    return written && locked && dashboardRead(id, HLSCL_LOCK, false) == 1 && readHardwareOffset(id, actual) &&
           actual == ticks;
}

bool CFeetechProtocol::calibrateHardwareOffset(uint8_t id) {
    if (!isConnected() || id > 253) return false;
    if (!dashboardWrite(id, HLSCL_TORQUE_ENABLE, 0, false) ||
        dashboardRead(id, HLSCL_TORQUE_ENABLE, false) != 0)
        return false;
    bool unlocked = dashboardWrite(id, HLSCL_LOCK, 0, false) && dashboardRead(id, HLSCL_LOCK, false) == 0;
    bool calibrated = unlocked && hls_servo_.Recal(id) == 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    // Relock even if the calibration acknowledgement was lost.
    bool locked = dashboardWrite(id, HLSCL_LOCK, 1, false);
    int16_t offset = 0;
    int position = dashboardRead(id, HLSCL_PRESENT_POSITION_L, true);
    return calibrated && locked && dashboardRead(id, HLSCL_LOCK, false) == 1 &&
           readHardwareOffset(id, offset) && std::abs(position - 2048) <= 8;
}

bool CFeetechProtocol::getPositionOffset(uint8_t id, int8_t& deviation) {
    int16_t ticks = 0;
    if (!readHardwareOffset(id, ticks)) return false;
    int value = std::lround(ticks * (360.0 / 4096.0) / 0.24);
    if (value < -128 || value > 127) return false;
    deviation = value;
    return true;
}

bool CFeetechProtocol::setPositionOffset(uint8_t id, int8_t deviation) {
    return writeHardwareOffset(id, std::lround(deviation * 0.24 * (4096.0 / 360.0)));
}

bool CFeetechProtocol::savePositionOffset(uint8_t id) {
    int16_t ticks = 0;
    return readHardwareOffset(id, ticks) && writeHardwareOffset(id, ticks);
}

bool CFeetechProtocol::getMotorSpeed(uint8_t id, int16_t& speed) {
    if (!is_connected_) return false;
    int res = hls_servo_.readWord(id, HLSCL_PRESENT_SPEED_L);
    if (res != -1) {
        speed = static_cast<int16_t>(hls::decodeSigned(res));
        return true;
    }
    return false;
}

bool CFeetechProtocol::getVoltage(uint8_t id, uint16_t& mv) {
    if (!is_connected_) return false;
    int res = hls_servo_.ReadVoltage(id);
    if (res != -1) {
        mv = static_cast<uint16_t>(res * 100);
        return true;
    }
    return false;
}

bool CFeetechProtocol::getTemperature(uint8_t id, uint8_t& deg_temp) {
    if (!is_connected_) return false;
    int res = hls_servo_.ReadTemper(id);
    if (res != -1) {
        deg_temp = static_cast<uint8_t>(res);
        return true;
    }
    return false;
}

bool CFeetechProtocol::getLedErrcode(uint8_t id, uint8_t& lederrcode) {
    if (!is_connected_) return false;
    int res = hls_servo_.Ping(id);
    if (res != -1 || hls_servo_.getLastError() == ERR_SERVO) {
        lederrcode = hls_servo_.getState();
        return true;
    }
    return false;
}

uint16_t CFeetechProtocol::convertTicksToFeetech(uint16_t ticks) const {
    double angle_deg = (static_cast<double>(ticks) - 500.0) * 0.24;
    double feetech_ticks = angle_deg * (4096.0 / 360.0) + 2048.0;
    if (feetech_ticks < 0.0) feetech_ticks = 0.0;
    if (feetech_ticks > 4095.0) feetech_ticks = 4095.0;
    return static_cast<uint16_t>(std::round(feetech_ticks));
}

uint16_t CFeetechProtocol::convertFeetechToTicks(uint16_t feetech_ticks) const {
    double angle_deg = (static_cast<double>(feetech_ticks) - 2048.0) * (360.0 / 4096.0);
    double ticks = angle_deg * (25.0 / 6.0) + 500.0;
    if (ticks < 0.0) ticks = 0.0;
    if (ticks > 1000.0) ticks = 1000.0;
    return static_cast<uint16_t>(std::round(ticks));
}

std::string CFeetechProtocol::lastErrorDetail() const {
    switch (hls_servo_.getLastError()) {
        case ERR_NO_REPLY:
            return "transaction timed out";
        case ERR_IO:
            return "serial I/O failed or disconnected";
        case ERR_CRC_CMP:
            return "response checksum mismatch";
        case ERR_SLAVE_ID:
            return "response came from a different servo";
        case ERR_BUFF_LEN:
            return "invalid packet length or buffer size";
        case ERR_SERVO:
            return "servo reported status " + std::to_string(hls_servo_.getState());
        default:
            return "register access failed";
    }
}

}  // namespace nira_movement
