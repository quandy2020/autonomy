#ifndef __CMD_PACKET_TRANSMITTER_H
#define __CMD_PACKET_TRANSMITTER_H

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <CmdPacket.h>
 
namespace Mmwave {

class CmdPacketTransmitter
{
public:
    using SendDataCallback = std::function<std::size_t(const u_int8_t* data, std::size_t size)>;
    using ReceiveDataCallback = std::function<std::size_t(u_int8_t* buffer, std::size_t size, int timeoutMs)>;

    CmdPacketTransmitter(const std::string& regConfigFile);
    ~CmdPacketTransmitter();

    bool SendStartCfgCmd();
    bool SendWriteSysCmd(const std::vector<uint8_t>& sysParaData);
    bool SendCascadingModeCmd();
    bool SendStartI2CTestCmd();
    bool SendFFTZeroFillCmd();
    bool SendWriteRegCmd();
    bool SendReadPdCalibrationCmd();
    bool SendFinishCfgCmd();
    bool SendRawCmd(CmdType cmdType, const std::vector<uint8_t>& data);

    void SetSendDataCallback(SendDataCallback callback);
    void SetReceiveDataCallback(ReceiveDataCallback callback);

private:
    const std::string regConfigFilePath_;
    SendDataCallback sendDataCallback_;
    ReceiveDataCallback receiveDataCallback_;

    std::size_t WaitForResponse(uint8_t* buffer, const std::size_t& expectedSize, int timeoutMs);
    std::size_t SendCmdPacket(const CmdPacket& cmdPacket);
    bool SendCmdAndWaitForResponse(const CmdPacket& cmdPacket, std::vector<uint8_t>& buffer, std::size_t expectedSize, int timeoutMs);

    bool ReadFileToBuffer(const std::string& filePath, std::vector<uint8_t>& buffer);
};

} // namespace Mmwave
#endif