#include <string>
#include <iostream>
#include <fstream>
#include <CmdPacketTransmitter.h>
#include <Cmd.h>
#include <Utils.h>

namespace Mmwave {

CmdPacketTransmitter::CmdPacketTransmitter(const std::string& regConfigFile)
    : regConfigFilePath_(regConfigFile),
      sendDataCallback_(nullptr),
      receiveDataCallback_(nullptr)
{
}

CmdPacketTransmitter::~CmdPacketTransmitter()
{
}

bool CmdPacketTransmitter::SendStartCfgCmd()
{
    SendCmdPacket(CmdPacket::CreateFromVector(CmdType::START_CFG_CMD,
        std::vector<uint8_t>{0x02, 0x00}));
    return true;
}

bool CmdPacketTransmitter::SendWriteSysCmd(const std::vector<uint8_t>& sysParaData)
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::WRITE_SYS_CMD, sysParaData),
        buffer, expectedSize, 1500);

    // std::cout << "WriteSysCmd, realResponseSize: " << realResponseSize << std::endl;
    return true;
}

bool CmdPacketTransmitter::SendCascadingModeCmd()
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::CASCADING_MODE_CMD,
        std::vector<uint8_t>{0x01, 0x00, 0x02, 0x00, 0x01, 0x00}), buffer, expectedSize, 1500);

    // std::cout << "CascadingModeCmd, realResponseSize: " << realResponseSize << std::endl;
    return true;
}

bool CmdPacketTransmitter::SendStartI2CTestCmd()
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::START_I2C_TEST_CMD,
        std::vector<uint8_t>{0x00, 0x00, 0x03, 0x00, 0x00, 0x00}), buffer, expectedSize, 1500);

    // std::cout << "StartI2CTestCmd, realResponseSize: " << realResponseSize << std::endl;
    return true;
}

bool CmdPacketTransmitter::SendFFTZeroFillCmd()
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::FFT_ZEROFILL_CMD,
        std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}), buffer, expectedSize, 1500);

    // std::cout << "FFTZeroFillCmd, realResponseSize: " << realResponseSize << std::endl;
    return true;
}

bool CmdPacketTransmitter::SendWriteRegCmd()
{
    std::ifstream file(regConfigFilePath_);
    uint16_t regAddr = 0, value = 0;
    bool eof = false;
    std::vector<uint8_t> buffer, responseBuffer;
    std::size_t expectedSize = 1000;

    buffer.push_back(0x00);
    buffer.push_back(0x20);

    if (file.is_open()) {
        while (ParseLine(file, regAddr, value, eof)) {
            if (buffer.size() + 4 > CmdPacket::MAX_PACKET_SIZE - 12) {
                SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::WRITE_REG_CMD, buffer),
                    responseBuffer, expectedSize, 1500);

                buffer.clear();
                buffer.push_back(0x00);
                buffer.push_back(0x20);
            }
            PushVector(buffer, htole16(regAddr));
            PushVector(buffer, htole16(value));
        }

        if (eof) {
            SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::WRITE_REG_CMD, buffer),
                responseBuffer, expectedSize, 1500);
            return true;
        } else {
            return false;
        }
    }

    return false;
}

bool CmdPacketTransmitter::SendReadPdCalibrationCmd()
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::READ_PD_CALIBRATION_CMD,
        std::vector<uint8_t>{0x10, 0x00, 0x11, 0x00, 0x12, 0x00, 0x13, 0x00}), buffer, expectedSize, 1500);
    
    return true;
}

bool CmdPacketTransmitter::SendFinishCfgCmd()
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(CmdType::FINISH_CFG_CMD,
        std::vector<uint8_t>{}), buffer, expectedSize, 1500);

    // std::cout << "FinishCfgCmd, realResponseSize:" << realResponseSize << std::endl;
    return true;
}

bool CmdPacketTransmitter::SendRawCmd(CmdType cmdType, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> buffer;
    std::size_t expectedSize = 1000;

    SendCmdAndWaitForResponse(CmdPacket::CreateFromVector(cmdType,
        data), buffer, expectedSize, 1500);

    // std::cout << "FinishCfgCmd, realResponseSize:" << realResponseSize << std::endl;
    return true;
}

void CmdPacketTransmitter::SetSendDataCallback(SendDataCallback callback)
{
    sendDataCallback_ = callback;
}

void CmdPacketTransmitter::SetReceiveDataCallback(ReceiveDataCallback callback)
{
    receiveDataCallback_ = callback;
}

// private 

std::size_t CmdPacketTransmitter::SendCmdPacket(const CmdPacket& cmdPacket)
{
    if (sendDataCallback_ != nullptr) {
        return sendDataCallback_(cmdPacket.GetPacketData().data(), cmdPacket.GetPacketData().size());
    }

    return 0;
}

std::size_t CmdPacketTransmitter::WaitForResponse(uint8_t* buffer, const std::size_t& expectedSize, int timeoutMs)
{
    if (receiveDataCallback_ != nullptr) {
        return receiveDataCallback_(buffer, expectedSize, timeoutMs);
    }

    return 0;
}

bool CmdPacketTransmitter::SendCmdAndWaitForResponse(const CmdPacket& cmdPacket, std::vector<uint8_t>& buffer, std::size_t expectedSize, int timeoutMs)
{

    SendCmdPacket(cmdPacket);
    buffer.resize(expectedSize);
    WaitForResponse(buffer.data(), expectedSize, timeoutMs);

    // std::cout << "Recv " << recv_len << " bytes" << std::endl;
    // return (expectedSize == WaitForResponse(buffer, expectedSize, timeoutMs));
    return true;


}

} // namespace Mmwave
