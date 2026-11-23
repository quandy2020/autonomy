#include <string>
#include <vector>
#include <iostream>
#include <Mmwave.h>

namespace Mmwave {

Mmwave::Mmwave(const std::string& portName, const std::string& regConfigFile)
    : cmdPacketTransmitter_(regConfigFile)
{
    try {
        serialPort_.open(portName, BaudRate::BAUD_921600, DataBits::EIGHT, Parity::NONE, StopBits::ONE, FlowControl::NONE);
        if (serialPort_.isOpen()) {
            cmdPacketTransmitter_.SetSendDataCallback([&](const uint8_t* data, std::size_t size) { 
                return serialPort_.write(data, size); 
            });
            cmdPacketTransmitter_.SetReceiveDataCallback([&](uint8_t* data, std::size_t size, int timeoutMs) {
                return serialPort_.read_timedwait(data, size, timeoutMs);
            });
        }
    } catch (const std::exception& e) {
        throw;
    }
}

Mmwave::~Mmwave()
{
    std::cout << "~Mmwave Begin" << std::endl;
    if (serialPort_.isOpen()) {
        serialPort_.close();
    }
    std::cout << "~Mmwave End" << std::endl;
}

bool Mmwave::StartSensor(const uint32_t chirpNum, uint32_t rangeBins)
{
    try {
        cmdPacketTransmitter_.SendStartCfgCmd();
        cmdPacketTransmitter_.SendStartCfgCmd();
        cmdPacketTransmitter_.SendStartCfgCmd();
        serialPort_.flush();

        cmdPacketTransmitter_.SendRawCmd(CmdType(0x0066), std::vector<uint8_t>{0x08, 0x00, 0x00, 0x00});

        cmdPacketTransmitter_.SendStartCfgCmd();
        serialPort_.flush();
        cmdPacketTransmitter_.SendStartCfgCmd();
        serialPort_.flush();

        cmdPacketTransmitter_.SendRawCmd(CmdType(0x0060), std::vector<uint8_t>{});

        cmdPacketTransmitter_.SendStartCfgCmd();
        serialPort_.flush();
        cmdPacketTransmitter_.SendStartCfgCmd();
        serialPort_.flush();

        cmdPacketTransmitter_.SendRawCmd(CmdType(0x0000), std::vector<uint8_t>{});

        cmdPacketTransmitter_.SendWriteSysCmd(std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
        cmdPacketTransmitter_.SendCascadingModeCmd();
        cmdPacketTransmitter_.SendWriteSysCmd(std::vector<uint8_t>{0x01, 0x00, 0x01, 0x00, 0x00, 0x00});
        cmdPacketTransmitter_.SendWriteSysCmd(std::vector<uint8_t>{0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x06, 0x00, 0x20, 0x22, 0x21, 0x23});
        cmdPacketTransmitter_.SendWriteSysCmd(std::vector<uint8_t>{0x07, 0x00, 0x00, 0x00, 0x00, 0x00});
        cmdPacketTransmitter_.SendWriteSysCmd(std::vector<uint8_t>{0x03, 0x00, 0x01, 0x00, 0x00, 0x00});
        cmdPacketTransmitter_.SendStartI2CTestCmd();
        cmdPacketTransmitter_.SendFFTZeroFillCmd();
        cmdPacketTransmitter_.SendWriteRegCmd();
        cmdPacketTransmitter_.SendReadPdCalibrationCmd();
        cmdPacketTransmitter_.SendFinishCfgCmd();
    } catch(std::exception& e) {
        throw;
    }

    rangeDataProcessor_.SetConfig(chirpNum, rangeBins);

    if (!rangeDataProcessor_.Start())
        return false;

    if (!serialStreamProcessor_.Start([this](const std::vector<uint8_t>& data) {
        rangeDataProcessor_.ReceiveRangeData(data);
    }))
        return false;

    if (!serialPort_.start_async_read([this](const std::vector<uint8_t>& data) 
    {
        serialStreamProcessor_.ReceiveStream(data); 
    }))
        return false;
    
    return true;
}

void Mmwave::StopSensor()
{
    serialPort_.stop_async_read();
    serialStreamProcessor_.Stop();
    rangeDataProcessor_.Stop();
}

} // namespace Mmwave
