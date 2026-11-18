/*******************************************************************************
 * Copyright (c) 2026 Christian Stein
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "handler/feetech/feetech_hls_servo.hpp"
#include "handler/servo_protocol.hpp"

namespace nira_movement {

class CFeetechProtocol : public CServoProtocol {
   public:
    CFeetechProtocol(std::shared_ptr<rclcpp::Node> node, const std::string& kDeviceName);
    ~CFeetechProtocol() override;

    void dashboard(uint8_t& confirmed_id, const nira_interfaces::srv::ServoDashboard::Request& kRequest,
                   nira_interfaces::srv::ServoDashboard::Response& response) override;

    // Register access is an HLS implementation detail and a test seam.
    virtual int dashboardRead(uint8_t id, uint8_t address, bool word);
    virtual bool dashboardWrite(uint8_t id, uint8_t address, uint16_t value, bool word);
    bool triggerConnection() override;
    void closeConnection() override;
    bool isConnected() override;

    bool getServoID(uint8_t id, uint8_t& answer) override;
    bool setServoID(uint8_t id, uint8_t new_id) override;
    bool getMode(uint8_t id, uint8_t& mode) override;
    bool setServoMode(uint8_t id) override;
    bool setMotorMode(uint8_t id, int16_t speed = 0) override;
    bool setTorque(uint8_t id, bool active) override;

    bool setPosition(uint8_t id, uint16_t pos, uint16_t time_ms) override;
    bool setRegPos(uint8_t id, uint16_t pos, uint16_t time_ms) override;
    bool actionStart(uint8_t id = SERVO_Broadcast_ID) override;
    bool moveStop(uint8_t id = SERVO_Broadcast_ID) override;
    bool getPosition(uint8_t id, int16_t& pos) override;
    bool getPositionLimits(uint8_t id, uint16_t& min_position, uint16_t& max_position) override;
    bool setPositionLimits(uint8_t id, uint16_t min_position, uint16_t max_position) override;
    bool readHardwareOffset(uint8_t id, int16_t& ticks) override;
    bool writeHardwareOffset(uint8_t id, int16_t ticks) override;
    bool calibrateHardwareOffset(uint8_t id) override;
    bool getPositionOffset(uint8_t id, int8_t& deviation) override;
    bool setPositionOffset(uint8_t id, int8_t deviation) override;
    bool savePositionOffset(uint8_t id) override;

    bool getMotorSpeed(uint8_t id, int16_t& speed) override;

    bool getVoltage(uint8_t id, uint16_t& mv) override;

    bool getTemperature(uint8_t id, uint8_t& deg_temp) override;

    bool getLedErrcode(uint8_t id, uint8_t& lederrcode) override;

   private:
    std::string lastErrorDetail() const;
    bool writePosition(uint8_t id, uint16_t pos, uint16_t time_ms, bool deferred);
    uint16_t convertTicksToFeetech(uint16_t ticks) const;
    uint16_t convertFeetechToTicks(uint16_t feetech_ticks) const;

    std::string device_name_;
    bool is_connected_ = false;
    FeetechHlsServo hls_servo_;
};

}  // namespace nira_movement
