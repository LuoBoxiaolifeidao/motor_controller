#include "MWD485Controller.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <algorithm>

static constexpr uint8_t FRAME_HEADER = 0x3E;

// command codes
static constexpr uint8_t CMD_DISABLE       = 0x80;
static constexpr uint8_t CMD_STOP          = 0x81;
static constexpr uint8_t CMD_ENABLE        = 0x88;
static constexpr uint8_t CMD_BRAKE         = 0x8C;
static constexpr uint8_t CMD_READ_ANGLE    = 0x92;
static constexpr uint8_t CMD_CLEAR_CIRCLE  = 0x93;
static constexpr uint8_t CMD_READ_SINGLE   = 0x94;
static constexpr uint8_t CMD_SET_ANGLE     = 0x95;
static constexpr uint8_t CMD_SET_ZERO      = 0x19;
static constexpr uint8_t CMD_READ_STATE1   = 0x9A;
static constexpr uint8_t CMD_CLEAR_ERROR   = 0x9B;
static constexpr uint8_t CMD_READ_STATE2   = 0x9C;
static constexpr uint8_t CMD_POS_MULTI1    = 0xA3;
static constexpr uint8_t CMD_POS_MULTI2    = 0xA4;
static constexpr uint8_t CMD_POS_SINGLE1   = 0xA5;
static constexpr uint8_t CMD_POS_SINGLE2   = 0xA6;
static constexpr uint8_t CMD_POS_INCR1     = 0xA7;
static constexpr uint8_t CMD_POS_INCR2     = 0xA8;
static constexpr uint8_t CMD_SPEED         = 0xA2;
static constexpr uint8_t CMD_READ_PARAM    = 0xC0;
static constexpr uint8_t CMD_WRITE_PARAM   = 0xC1;

MWD485Controller::MWD485Controller(int motor_id, const std::string& port, int baudrate)
    : motor_id_(motor_id)
{
    serial_ = SerialPort::create(port, baudrate);
}

MWD485Controller::~MWD485Controller() = default;

std::vector<uint8_t> MWD485Controller::buildFrame(uint8_t cmd, const std::vector<uint8_t>& data) const
{
    std::vector<uint8_t> frame;
    frame.reserve(5 + data.size() + 1);

    uint8_t len = static_cast<uint8_t>(data.size());
    uint8_t hdrCsum = (FRAME_HEADER + cmd + motor_id_ + len) & 0xFF;

    frame.push_back(FRAME_HEADER);
    frame.push_back(cmd);
    frame.push_back(static_cast<uint8_t>(motor_id_));
    frame.push_back(len);
    frame.push_back(hdrCsum);

    if (!data.empty()) {
        frame.insert(frame.end(), data.begin(), data.end());
        uint8_t dataCsum = 0;
        for (auto b : data) dataCsum += b;
        frame.push_back(dataCsum);
    }

    return frame;
}

bool MWD485Controller::parseResponse(const std::vector<uint8_t>& raw, uint8_t expectedCmd,
                                      std::vector<uint8_t>& data) const
{
    if (raw.size() < 5)
        return false;

    if (raw[0] != FRAME_HEADER || raw[1] != expectedCmd || raw[2] != motor_id_)
        return false;

    uint8_t len = raw[3];
    uint8_t hdrCsum = (raw[0] + raw[1] + raw[2] + raw[3]) & 0xFF;
    if (hdrCsum != raw[4])
        return false;

    if (len > 0) {
        if (raw.size() < 5 + len + 1)
            return false;

        data.assign(raw.begin() + 5, raw.begin() + 5 + len);

        uint8_t dataCsum = 0;
        for (int i = 0; i < len; i++)
            dataCsum += data[i];
        if (dataCsum != raw[5 + len])
            return false;
    }

    return true;
}

static void hexDump(const char* label, const std::vector<uint8_t>& data) {
    std::cerr << "  " << label << " [" << data.size() << "]:";
    for (auto b : data) fprintf(stderr, " %02X", b);
    std::cerr << "\n";
}

std::vector<uint8_t> MWD485Controller::sendAndRecv(uint8_t cmd,
                                                    const std::vector<uint8_t>& txData,
                                                    int timeoutMs, int retries)
{
    auto frame = buildFrame(cmd, txData);

    for (int i = 0; i < retries; ++i) {
        // hexDump("TX", frame);
        auto response = serial_->transaction(frame, timeoutMs);
        // hexDump("RX", response);
        std::vector<uint8_t> data;
        if (parseResponse(response, cmd, data))
            return data;
        // std::cerr << "  parse FAIL\n";
        if (i < retries - 1)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // std::cerr << "  sendAndRecv cmd=0x" << std::hex << (int)cmd << std::dec << " FAILED\n";
    return {};
}

void MWD485Controller::sendMultiTurnPosition(int32_t angle_centideg, uint16_t maxSpeed_dps)
{
    int64_t angle = static_cast<int64_t>(angle_centideg);
    uint32_t speed = static_cast<uint32_t>(maxSpeed_dps) * 100;

    std::vector<uint8_t> data;
    data.reserve(12);
    for (int i = 0; i < 8; i++)
        data.push_back((angle >> (i * 8)) & 0xFF);
    for (int i = 0; i < 4; i++)
        data.push_back((speed >> (i * 8)) & 0xFF);

    auto frame = buildFrame(CMD_POS_MULTI2, data);
    serial_->send(frame);
}

// === Motor Control ===

void MWD485Controller::enable()
{
    sendAndRecv(CMD_ENABLE, {}, 100, 3);
}

void MWD485Controller::disable()
{
    sendAndRecv(CMD_DISABLE, {}, 100, 3);
}

void MWD485Controller::stop()
{
    sendAndRecv(CMD_STOP, {}, 100, 3);
}

void MWD485Controller::brakeRelease()
{
    sendAndRecv(CMD_BRAKE, {0x01}, 100, 3);
}

void MWD485Controller::brakeLock()
{
    sendAndRecv(CMD_BRAKE, {0x00}, 100, 3);
}

int MWD485Controller::getBrakeState()
{
    auto data = sendAndRecv(CMD_BRAKE, {0x10}, 200, 3);
    if (data.size() >= 1)
        return data[0];
    return -1;
}

void MWD485Controller::clearError()
{
    sendAndRecv(CMD_CLEAR_ERROR, {}, 100, 3);
}

void MWD485Controller::setAcceleration(int32_t accel_dps2)
{
    // param 0x26: inputSpeedRamp (int32_t, 4 bytes LE)
    std::vector<uint8_t> val(6, 0);
    memcpy(val.data(), &accel_dps2, 4);
    writeParam(0x26, val);
}

void MWD485Controller::setSpeedLimit(int32_t speedLimit)
{
    std::vector<uint8_t> val(6, 0);
    memcpy(val.data(), &speedLimit, 4);
    writeParam(0x20, val);
}

void MWD485Controller::setAngleLimit(int32_t angleLimit)
{
    std::vector<uint8_t> val(6, 0);
    memcpy(val.data(), &angleLimit, 4);
    writeParam(0x22, val);
}

int32_t MWD485Controller::getAngleLimit()
{
    auto data = readParam(0x22);
    if (data.size() < 4) return 0;
    int32_t limit = 0;
    memcpy(&limit, data.data(), 4);
    return limit;
}

void MWD485Controller::setZeroToROM()
{
    sendAndRecv(CMD_SET_ZERO, {}, 200, 3);
}

// === Movement ===

void MWD485Controller::multiTurnPosition(int32_t angle_centideg, uint16_t maxSpeed_dps)
{
    int64_t angle = static_cast<int64_t>(angle_centideg);
    uint32_t speed = static_cast<uint32_t>(maxSpeed_dps) * 100;
    multiTurnPosition(angle, speed);
}

void MWD485Controller::multiTurnPosition(int64_t angle, uint32_t maxSpeed)
{
    std::vector<uint8_t> data;
    data.reserve(12);

    for (int i = 0; i < 8; i++)
        data.push_back((angle >> (i * 8)) & 0xFF);

    for (int i = 0; i < 4; i++)
        data.push_back((maxSpeed >> (i * 8)) & 0xFF);

    sendAndRecv(CMD_POS_MULTI2, data, 200, 3);
}

void MWD485Controller::incrementalPosition(int32_t delta, uint32_t maxSpeed)
{
    std::vector<uint8_t> data;
    data.reserve(8);

    for (int i = 0; i < 4; i++)
        data.push_back((delta >> (i * 8)) & 0xFF);

    uint32_t speedScaled = maxSpeed * 100;
    for (int i = 0; i < 4; i++)
        data.push_back((speedScaled >> (i * 8)) & 0xFF);

    sendAndRecv(CMD_POS_INCR2, data, 200, 3);
}

void MWD485Controller::speedControl(int32_t speed)
{
    std::vector<uint8_t> data(4);
    memcpy(data.data(), &speed, 4);
    sendAndRecv(CMD_SPEED, data, 200, 3);
}

void MWD485Controller::setAnyAngle(int32_t angle)
{
    std::vector<uint8_t> data(4);
    memcpy(data.data(), &angle, 4);
    sendAndRecv(CMD_SET_ANGLE, data, 200, 3);
}

// === State Reads ===

void MWD485Controller::refreshState()
{
    auto data1 = sendAndRecv(CMD_READ_STATE1, {}, 200, 3);
    if (data1.size() >= 7) {
        cacheTemp_       = static_cast<int8_t>(data1[0]);
        memcpy(&cacheVoltage_, data1.data() + 1, 2);
        memcpy(&cacheCurrent_, data1.data() + 3, 2);
        cacheMotorState_ = data1[5];
        cacheErrorState_ = data1[6];
    }

    auto data2 = sendAndRecv(CMD_READ_STATE2, {}, 200, 3);
    if (data2.size() >= 8) {
        memcpy(&cacheSpeed_,   data2.data() + 3, 2);
        memcpy(&cacheEncoder_, data2.data() + 5, 2);
        cacheTemp_ = static_cast<int8_t>(data2[0]);
    }

    auto dataAngle = sendAndRecv(CMD_READ_ANGLE, {}, 200, 3);
    if (dataAngle.size() >= 8) {
        memcpy(&cacheAngle_, dataAngle.data(), 8);
    }
}

int32_t MWD485Controller::getAcceleration()
{
    auto data = readParam(0x26);
    if (data.size() >= 4) {
        int32_t accel = 0;
        memcpy(&accel, data.data(), 4);
        return accel;
    }
    return 0;
}

// === Control Parameters ===

void MWD485Controller::writeParam(uint8_t paramId, const std::vector<uint8_t>& value)
{
    std::vector<uint8_t> data;
    data.reserve(1 + value.size());
    data.push_back(paramId);
    data.insert(data.end(), value.begin(), value.end());
    sendAndRecv(CMD_WRITE_PARAM, data, 200, 3);
}

std::vector<uint8_t> MWD485Controller::readParam(uint8_t paramId)
{
    return sendAndRecv(CMD_READ_PARAM, {paramId}, 200, 3);
}
