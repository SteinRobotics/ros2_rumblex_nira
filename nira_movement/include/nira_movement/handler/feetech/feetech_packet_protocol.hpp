#pragma once

#include "feetech_protocol_definitions.hpp"

// Packet framing shared by the HLS operations and the serial transport.
class FeetechPacketProtocol {
   public:
    FeetechPacketProtocol() = default;
    explicit FeetechPacketProtocol(u8 byte_order) : byte_order_(byte_order) {
    }
    FeetechPacketProtocol(u8 byte_order, u8 response_level)
        : response_level_(response_level), byte_order_(byte_order) {
    }
    virtual ~FeetechPacketProtocol() = default;
    int genWrite(u8 id, u8 address, u8* data, u8 size);
    int regWrite(u8 id, u8 address, u8* data, u8 size);
    int RegWriteAction(u8 id = 0xfe);
    int writeByte(u8 id, u8 address, u8 value);
    int writeWord(u8 id, u8 address, u16 value);
    int Read(u8 id, u8 address, u8* data, u8 size);
    int readByte(u8 id, u8 address);
    int readWord(u8 id, u8 address);
    int Ping(u8 id);
    int Recal(u8 id);
    u8 getState() const {
        return servo_status_;
    }
    u8 getLastError() const {
        return error_code_;
    }

    u8 response_level_ = 1;  // Expect replies except for broadcasts.
    u8 byte_order_ = 0;      // Zero selects little-endian register words.
    u8 servo_status_ = 0;
    u8 error_code_ = 0;

   protected:
    virtual int writeSCS(unsigned char* data, int size) = 0;
    virtual int readSCS(unsigned char* data, int size) = 0;
    virtual int writeSCS(unsigned char value) = 0;
    virtual void rFlushSCS() = 0;
    virtual bool wFlushSCS() = 0;
    void Host2SCS(u8* low, u8* high, u16 value);
    u16 SCS2Host(u8 low, u8 high);

   private:
    bool send(u8 id, u8 instruction, u8* parameters, int size);
    bool receive(u8 id, u8* data, u8 size);
    int writeRegisters(u8 id, u8 address, u8* data, u8 size, u8 instruction);
    int acknowledge(u8 id);
};
