#include "feetech_packet_protocol.hpp"

#include <algorithm>
#include <array>

void FeetechPacketProtocol::Host2SCS(u8* low, u8* high, u16 value) {
    *low = byte_order_ ? value >> 8 : value & 0xff;
    *high = byte_order_ ? value & 0xff : value >> 8;
}
u16 FeetechPacketProtocol::SCS2Host(u8 low, u8 high) {
    return byte_order_ ? (low << 8) | high : (high << 8) | low;
}

bool FeetechPacketProtocol::send(u8 id, u8 instruction, u8* parameters, int size) {
    error_code_ = 0;
    servo_status_ = 0;
    if (id > 254 || size < 0 || size > 253 || (size && !parameters)) {
        error_code_ = ERR_BUFF_LEN;
        return false;
    }
    rFlushSCS();  // Starts the transport's deadline, shared by send and receive.
    if (error_code_) return false;
    u8 header[] = {0xff, 0xff, id, static_cast<u8>(size + 2), instruction};
    u8 sum = id + size + 2 + instruction;
    for (int i = 0; i < size; ++i) sum += parameters[i];
    if (writeSCS(header, sizeof(header)) != sizeof(header) || (size && writeSCS(parameters, size) != size) ||
        writeSCS(static_cast<u8>(~sum)) != 1 || !wFlushSCS()) {
        if (!error_code_) error_code_ = ERR_IO;
        return false;
    }
    return true;
}

bool FeetechPacketProtocol::receive(u8 id, u8* data, u8 size) {
    auto read_exact = [&](u8* output, int count) {
        if (readSCS(output, count) == count) return true;
        if (!error_code_) error_code_ = ERR_NO_REPLY;
        return false;
    };
    unsigned consecutive = 0;
    for (unsigned scanned = 0; consecutive < 2; ++scanned) {
        if (scanned == 32) {
            error_code_ = ERR_BUFF_LEN;
            return false;
        }
        u8 byte = 0;
        if (!read_exact(&byte, 1)) return false;
        consecutive = byte == 0xff ? consecutive + 1 : 0;
    }
    u8 header[3];
    if (!read_exact(header, sizeof(header))) return false;
    if (header[0] != id) {
        error_code_ = ERR_SLAVE_ID;
        return false;
    }
    if (header[1] != size + 2) {
        error_code_ = ERR_BUFF_LEN;
        return false;
    }
    std::array<u8, 253> payload{};
    if (size && !read_exact(payload.data(), size)) return false;
    u8 checksum = 0;
    if (!read_exact(&checksum, 1)) return false;
    u8 sum = header[0] + header[1] + header[2];
    for (unsigned i = 0; i < size; ++i) sum += payload[i];
    if (checksum != static_cast<u8>(~sum)) {
        error_code_ = ERR_CRC_CMP;
        return false;
    }
    servo_status_ = header[2];
    if (servo_status_) {
        error_code_ = ERR_SERVO;
        return false;
    }
    if (size) std::copy_n(payload.begin(), size, data);
    return true;
}

int FeetechPacketProtocol::acknowledge(u8 id) {
    return id == 254 || !response_level_ || receive(id, nullptr, 0) ? 1 : 0;
}
int FeetechPacketProtocol::writeRegisters(u8 id, u8 address, u8* data, u8 size, u8 instruction) {
    if (size > 252 || (size && !data)) {
        error_code_ = ERR_BUFF_LEN;
        return 0;
    }
    std::array<u8, 253> parameters{};
    parameters[0] = address;
    if (size) std::copy_n(data, size, parameters.begin() + 1);
    return send(id, instruction, parameters.data(), size + 1) ? acknowledge(id) : 0;
}
int FeetechPacketProtocol::genWrite(u8 id, u8 address, u8* data, u8 size) {
    return writeRegisters(id, address, data, size, INST_WRITE);
}
int FeetechPacketProtocol::regWrite(u8 id, u8 address, u8* data, u8 size) {
    return writeRegisters(id, address, data, size, INST_REG_WRITE);
}
int FeetechPacketProtocol::RegWriteAction(u8 id) {
    return send(id, INST_REG_ACTION, nullptr, 0) ? acknowledge(id) : 0;
}
int FeetechPacketProtocol::writeByte(u8 id, u8 address, u8 value) {
    return genWrite(id, address, &value, 1);
}
int FeetechPacketProtocol::writeWord(u8 id, u8 address, u16 value) {
    u8 data[2];
    Host2SCS(data, data + 1, value);
    return genWrite(id, address, data, 2);
}
int FeetechPacketProtocol::Read(u8 id, u8 address, u8* data, u8 size) {
    if (id > 253 || !data || !size || size > 253) {
        error_code_ = ERR_BUFF_LEN;
        return 0;
    }
    u8 parameters[] = {address, size};
    return send(id, INST_READ, parameters, sizeof(parameters)) && receive(id, data, size) ? size : 0;
}
int FeetechPacketProtocol::readByte(u8 id, u8 address) {
    u8 value = 0;
    return Read(id, address, &value, 1) == 1 ? value : -1;
}
int FeetechPacketProtocol::readWord(u8 id, u8 address) {
    u8 data[2];
    return Read(id, address, data, 2) == 2 ? SCS2Host(data[0], data[1]) : -1;
}
int FeetechPacketProtocol::Ping(u8 id) {
    if (id > 253) {
        error_code_ = ERR_BUFF_LEN;
        return -1;
    }
    return send(id, INST_PING, nullptr, 0) && receive(id, nullptr, 0) ? id : -1;
}
int FeetechPacketProtocol::Recal(u8 id) {
    return send(id, INST_CAL, nullptr, 0) ? acknowledge(id) : 0;
}
