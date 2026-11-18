#include "handler/servo_controller.hpp"

namespace nira_movement {
void CServoController::dashboard(const nira_interfaces::srv::ServoDashboard::Request& q,
                                 nira_interfaces::srv::ServoDashboard::Response& r) {
    using Request = nira_interfaces::srv::ServoDashboard::Request;
    using Response = nira_interfaces::srv::ServoDashboard::Response;
    r = Response{};
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
    if (q.operation == Request::ID) {
        for (const auto& [idx, other] : servos_) {
            (void)idx;
            if (other.getSerialID() == q.new_id) {
                r.detail = "ID already configured";
                return;
            }
        }
    }
    uint8_t id = servo.getSerialID();
    protocol_->dashboard(id, q, r);
    servo.setSerialID(id);
    if (r.outcome == Response::VERIFIED && q.operation != Request::READ_OFFSET &&
        q.operation != Request::SAVE_OFFSET && q.operation != Request::CALIBRATE_OFFSET) {
        r.state.name = q.name;
        r.state.header.stamp = node_->now();
    }
}
}  // namespace nira_movement
