/*
 * SMS_STS_Shared.h
 * SMS_STS 子类: 通过 SerialPort 单例通信, 而非独立打开串口 fd
 *
 * 解决的问题:
 *   robot_controller 中 MWD485(SerialPort) 和 FT(SMS_STS) 各自独立 open
 *   /dev/ttysWK0, 导致 3 个 fd 竞争同一 RS-485 总线。本类复用 SerialPort
 *   单例, 通信通过 SerialPort 内部 mutex 排队。
 *
 * 用法:
 *   auto sp = SerialPort::create(ft_cfg_.port, ft_cfg_.baudrate);
 *   auto ft = std::make_shared<SMS_STS_Shared>(sp);
 *   ft->begin(ft_cfg_.baudrate, nullptr);
 *   // 之后所有 SMS_STS 方法均可正常使用
 */

#ifndef _SMS_STS_SHARED_H
#define _SMS_STS_SHARED_H

#include "SMS_STS.h"
#include "SerialPort.h"
#include <memory>
#include <vector>
#include <cstring>
#include <algorithm>

class SMS_STS_Shared : public SMS_STS {
private:
    std::shared_ptr<SerialPort> sp_;

protected:
    // 写缓冲 (沿用父类 SCSerial 的 txBuf 缓冲机制)
    using SCSerial::txBuf;
    using SCSerial::txBufLen;
    using SCSerial::fd;
    using SCSerial::IOTimeOut;

    int writeSCS(unsigned char* nDat, int nLen) override {
        while (nLen--) txBuf[txBufLen++] = *nDat++;
        return txBufLen;
    }

    int writeSCS(unsigned char bDat) override {
        txBuf[txBufLen++] = bDat;
        return txBufLen;
    }

    void wFlushSCS() override {
        if (txBufLen == 0) return;
        sp_->writeRaw(std::vector<uint8_t>(txBuf, txBuf + txBufLen));
        txBufLen = 0;
    }

    int readSCS(unsigned char* nDat, int nLen, unsigned long TimeOut) override {
        auto rv = sp_->readBytes(nLen, static_cast<int>(TimeOut));
        if (rv.empty()) return 0;
        int copyLen = std::min(nLen, static_cast<int>(rv.size()));
        std::memcpy(nDat, rv.data(), copyLen);
        return copyLen;
    }

    int readSCS(unsigned char* nDat, int nLen) override {
        return readSCS(nDat, nLen, IOTimeOut);
    }

    void rFlushSCS() override {
        sp_->flushInput();
    }

public:
    explicit SMS_STS_Shared(std::shared_ptr<SerialPort> sp)
        : SMS_STS(), sp_(std::move(sp))
    {
        this->fd = sp_->getFd();
        this->txBufLen = 0;
        this->IOTimeOut = 100;
    }

    // 覆盖 begin: 不复 open 端口, 在已有 fd 上设置波特率
    bool begin(int baudRate, const char* /*serialPort*/) {
        if (!sp_) return false;
        this->fd = sp_->getFd();
        return setBaudRate(baudRate) == 1;
    }

    // 覆盖 end: 不关闭 fd (SerialPort 管理生命周期)
    void end() {
        this->fd = -1;
    }
};

#endif
