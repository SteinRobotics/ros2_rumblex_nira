#include <stdexcept>

#include "handler/servo_controller.hpp"

namespace nira_movement {
void CServoController::dashboard(const nira_interfaces::srv::ServoDashboard::Request& q,
                                 nira_interfaces::srv::ServoDashboard::Response& r) {
    using Request = nira_interfaces::srv::ServoDashboard::Request;
    using Response = nira_interfaces::srv::ServoDashboard::Response;
    r.outcome = Response::REJECTED;
    if (q.operation == Request::LIST) {
        for (const auto& [idx, servo] : servos_) {
            (void)idx;
            r.names.push_back(servo.getName());
            r.ids.push_back(servo.getSerialID());
        }
        r.outcome = Response::VERIFIED;
        return;
    }
    if (!connection_ready_ || !protocol_) {
        r.detail = "Serial bus unavailable; correct servo.serial_port and restart movement node";
        return;
    }
    auto found = name_to_idx_.find(q.name);
    if (found == name_to_idx_.end()) {
        r.detail = "Unknown configured servo";
        return;
    }
    auto& servo = servos_.at(found->second);
    uint8_t id = servo.getSerialID();
    auto read = [&](uint8_t address, bool word = true) {
        int value = protocol_->dashboardRead(id, address, word);
        if (value < 0) throw std::runtime_error("Register read failed or transport unsupported");
        return value;
    };
    auto write = [&](uint8_t address, uint16_t value, bool word = true) {
        r.outcome = Response::UNVERIFIED;
        if (!protocol_->dashboardWrite(id, address, value, word))
            throw std::runtime_error("Write attempted; response failed. Inspect feedback before retrying");
    };
    auto verify = [&](uint8_t address, int value, bool word = true) {
        if (read(address, word) != value)
            throw std::runtime_error("Write attempted but readback differs; check EEPROM protection");
    };
    try {
        if (q.operation == Request::READ_OFFSET || q.operation == Request::SAVE_OFFSET ||
            q.operation == Request::CALIBRATE_OFFSET) {
            int16_t offset = 0;
            if (!protocol_->readHardwareOffset(id, offset))
                throw std::runtime_error("Hardware offset unsupported or read failed");
            if (q.operation == Request::SAVE_OFFSET) {
                if (q.offset_ticks < -4095 || q.offset_ticks > 4095)
                    throw std::runtime_error("HLS offset must be within -4095..4095 ticks");
                r.outcome = Response::UNVERIFIED;
                if (!protocol_->writeHardwareOffset(id, q.offset_ticks))
                    throw std::runtime_error(
                        "Offset write attempted; verify offset and EEPROM lock. Torque remains off");
            } else if (q.operation == Request::CALIBRATE_OFFSET) {
                r.outcome = Response::UNVERIFIED;
                if (!protocol_->calibrateHardwareOffset(id))
                    throw std::runtime_error(
                        "Calibration attempted; midpoint/offset/lock not verified. Torque remains off");
            }
            if (!protocol_->readHardwareOffset(id, r.offset_ticks))
                throw std::runtime_error("Offset readback failed");
            r.outcome = Response::VERIFIED;
            r.detail = q.operation == Request::READ_OFFSET
                           ? "Offset read from servo"
                           : "Offset stored in servo EEPROM and lock verified; torque remains off";
            return;
        }
        if (q.operation == Request::ENABLE_TORQUE) {
            int position = read(56);
            if (position > 4095) throw std::runtime_error("Position outside single-turn range");
            write(42, position);
            verify(42, position);
            write(40, 1, false);
            verify(40, 1, false);
        } else if (q.operation == Request::MOVE || q.operation == Request::PROFILE) {
            if (q.speed > 3400 || q.acc > 150) throw std::runtime_error("Invalid motion profile");
            if (q.operation == Request::MOVE) {
                int lo = read(9), hi = read(11);
                if (!(lo < hi && hi <= 4095)) {
                    lo = 0;
                    hi = 4095;
                }
                if (q.goal < lo || q.goal > hi) throw std::runtime_error("Target outside travel limits");
            }
            write(41, q.acc, false);
            write(46, q.speed);
            verify(41, q.acc, false);
            verify(46, q.speed);
            if (q.operation == Request::MOVE) {
                write(42, q.goal);
                verify(42, q.goal);
            }
        } else if (q.operation == Request::HOLD) {
            int position = read(56);
            if (position > 4095) throw std::runtime_error("Position outside single-turn range");
            write(42, position);
            verify(42, position);
        } else if (q.operation == Request::LIMITS) {
            if (q.minimum >= q.maximum || q.maximum > 4095) throw std::runtime_error("Invalid travel limits");
            write(9, q.minimum);
            write(11, q.maximum);
            verify(9, q.minimum);
            verify(11, q.maximum);
        } else if (q.operation == Request::ID) {
            if (q.new_id > 253) throw std::runtime_error("Invalid ID");
            for (const auto& [idx, other] : servos_) {
                (void)idx;
                if (other.getSerialID() == q.new_id) throw std::runtime_error("ID already configured");
            }
            uint8_t answer = 0;
            if (protocol_->getServoID(q.new_id, answer)) throw std::runtime_error("ID already responds");
            write(5, q.new_id, false);
            id = q.new_id;
            verify(5, id, false);
            servo.setSerialID(id);
        } else if (q.operation != Request::STATE) {
            throw std::runtime_error("Unsupported dashboard operation");
        }
        // Read a complete fresh sample; failures never masquerade as fresh feedback.
        auto& s = r.state;
        s.name = q.name;
        s.id = id;
        s.position = read(56);
        s.goal = read(42);
        int velocity = read(58);
        s.velocity = (velocity & 0x8000) ? -(velocity & 0x7fff) : velocity;
        s.load = read(60);
        s.voltage = read(62, false) / 10.0F;
        s.temperature = read(63, false);
        s.status = read(65, false);
        s.minimum = read(9);
        s.maximum = read(11);
        s.speed = read(46);
        s.acc = read(41, false);
        s.baud_code = read(6, false);
        s.header.stamp = node_->now();
        r.outcome = Response::VERIFIED;
        r.detail =
            q.operation == Request::ID ? "Verified; update servo.serial_ids in YAML separately" : "Verified";
    } catch (const std::exception& e) {
        r.detail = e.what();
    }
}
}  // namespace nira_movement
