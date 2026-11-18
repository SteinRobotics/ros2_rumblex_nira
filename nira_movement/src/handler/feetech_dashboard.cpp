#include <stdexcept>

#include "handler/feetech_protocol.hpp"

namespace nira_movement {
void CFeetechProtocol::dashboard(uint8_t& confirmed_id,
                                 const nira_interfaces::srv::ServoDashboard::Request& kRequest,
                                 nira_interfaces::srv::ServoDashboard::Response& response) {
    using Request = nira_interfaces::srv::ServoDashboard::Request;
    using Response = nira_interfaces::srv::ServoDashboard::Response;
    response = Response{};
    response.outcome = Response::REJECTED;
    uint8_t id = confirmed_id;
    if (!isConnected() || id > 253) {
        response.detail = "HLS bus unavailable or invalid servo ID";
        return;
    }
    auto read = [&](uint8_t address, bool word = true) {
        int value = dashboardRead(id, address, word);
        if (value < 0) throw std::runtime_error("Register read failed: " + lastErrorDetail());
        return value;
    };
    auto write = [&](uint8_t address, uint16_t value, bool word = true) {
        response.outcome = Response::UNVERIFIED;
        if (!dashboardWrite(id, address, value, word))
            throw std::runtime_error("Write attempted: " + lastErrorDetail() +
                                     "; inspect feedback before retrying");
    };
    auto verify = [&](uint8_t address, int value, bool word = true) {
        if (read(address, word) != value)
            throw std::runtime_error("Write attempted but readback differs; check EEPROM protection");
    };
    try {
        if (kRequest.operation == Request::READ_OFFSET || kRequest.operation == Request::SAVE_OFFSET ||
            kRequest.operation == Request::CALIBRATE_OFFSET) {
            int16_t offset = 0;
            if (!readHardwareOffset(id, offset))
                throw std::runtime_error("Hardware offset unsupported or read failed");
            if (kRequest.operation == Request::SAVE_OFFSET) {
                if (kRequest.offset_ticks < -4095 || kRequest.offset_ticks > 4095)
                    throw std::runtime_error("HLS offset must be within -4095..4095 ticks");
                response.outcome = Response::UNVERIFIED;
                if (!writeHardwareOffset(id, kRequest.offset_ticks))
                    throw std::runtime_error(
                        "Offset write attempted; verify offset and EEPROM lock. Torque remains off");
            } else if (kRequest.operation == Request::CALIBRATE_OFFSET) {
                response.outcome = Response::UNVERIFIED;
                if (!calibrateHardwareOffset(id))
                    throw std::runtime_error(
                        "Calibration attempted; midpoint/offset/lock not verified. Torque remains off");
            }
            if (!readHardwareOffset(id, response.offset_ticks))
                throw std::runtime_error("Offset readback failed");
            response.outcome = Response::VERIFIED;
            response.detail = kRequest.operation == Request::READ_OFFSET
                                  ? "Offset read from servo"
                                  : "Offset stored in servo EEPROM and lock verified; torque remains off";
            return;
        }
        if (kRequest.operation == Request::ENABLE_TORQUE || kRequest.operation == Request::HOLD) {
            int position = read(HLSCL_PRESENT_POSITION_L);
            if (position > 4095) throw std::runtime_error("Position outside single-turn range");
            write(HLSCL_GOAL_POSITION_L, position);
            verify(HLSCL_GOAL_POSITION_L, position);
            if (kRequest.operation == Request::ENABLE_TORQUE) {
                write(HLSCL_TORQUE_ENABLE, 1, false);
                verify(HLSCL_TORQUE_ENABLE, 1, false);
            }
        } else if (kRequest.operation == Request::MOVE || kRequest.operation == Request::PROFILE) {
            if (kRequest.speed > 3400 || kRequest.acc > 150)
                throw std::runtime_error("Invalid motion profile");
            if (kRequest.operation == Request::MOVE) {
                int lo = read(HLSCL_MIN_ANGLE_LIMIT_L), hi = read(HLSCL_MAX_ANGLE_LIMIT_L);
                if (!(lo < hi && hi <= 4095))
                    throw std::runtime_error("Invalid travel limits; refusing motion");
                if (kRequest.goal < lo || kRequest.goal > hi)
                    throw std::runtime_error("Target outside travel limits");
            }
            write(HLSCL_ACC, kRequest.acc, false);
            write(HLSCL_GOAL_SPEED_L, kRequest.speed);
            verify(HLSCL_ACC, kRequest.acc, false);
            verify(HLSCL_GOAL_SPEED_L, kRequest.speed);
            if (kRequest.operation == Request::MOVE) {
                write(HLSCL_GOAL_POSITION_L, kRequest.goal);
                verify(HLSCL_GOAL_POSITION_L, kRequest.goal);
            }
        } else if (kRequest.operation == Request::LIMITS) {
            if (kRequest.minimum >= kRequest.maximum || kRequest.maximum > 4095)
                throw std::runtime_error("Invalid travel limits");
            write(HLSCL_MIN_ANGLE_LIMIT_L, kRequest.minimum);
            write(HLSCL_MAX_ANGLE_LIMIT_L, kRequest.maximum);
            verify(HLSCL_MIN_ANGLE_LIMIT_L, kRequest.minimum);
            verify(HLSCL_MAX_ANGLE_LIMIT_L, kRequest.maximum);
        } else if (kRequest.operation == Request::ID) {
            if (kRequest.new_id > 253) throw std::runtime_error("Invalid ID");
            uint8_t answer = 0;
            if (getServoID(kRequest.new_id, answer)) throw std::runtime_error("ID already responds");
            write(HLSCL_ID, kRequest.new_id, false);
            id = kRequest.new_id;
            verify(HLSCL_ID, id, false);
            confirmed_id = id;
        } else if (kRequest.operation != Request::STATE) {
            throw std::runtime_error("Unsupported dashboard operation");
        }
        // Read a complete fresh sample; failures never masquerade as fresh feedback.
        nira_interfaces::msg::ServoDashboardState s;
        s.id = id;
        s.position = read(HLSCL_PRESENT_POSITION_L);
        s.goal = read(HLSCL_GOAL_POSITION_L);
        int velocity = read(HLSCL_PRESENT_SPEED_L);
        s.velocity = hls::decodeSigned(velocity);
        s.load = read(HLSCL_PRESENT_LOAD_L);
        s.voltage = read(HLSCL_PRESENT_VOLTAGE, false) / 10.0F;
        s.temperature = read(HLSCL_PRESENT_TEMPERATURE, false);
        s.status = read(HLSCL_STATUS, false);
        s.minimum = read(HLSCL_MIN_ANGLE_LIMIT_L);
        s.maximum = read(HLSCL_MAX_ANGLE_LIMIT_L);
        s.speed = read(HLSCL_GOAL_SPEED_L);
        s.acc = read(HLSCL_ACC, false);
        s.baud_code = read(HLSCL_BAUD_RATE, false);
        response.state = s;
        response.outcome = Response::VERIFIED;
        response.detail = kRequest.operation == Request::ID
                              ? "Verified; update servo.serial_ids in YAML separately"
                              : "Verified";
    } catch (const std::exception& kE) {
        response.detail = kE.what();
    }
    return;
}
}  // namespace nira_movement
