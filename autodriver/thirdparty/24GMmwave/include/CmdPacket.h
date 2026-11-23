#ifndef __CMD_PACKET_H
#define __CMD_PACKET_H

#include <cstdint>
#include <vector>
#include <Cmd.h>

namespace Mmwave {

class CmdPacket
{
public:
    // 构造函数
    CmdPacket();
    
    // 使用std::vector<uint8_t>生成CmdPacket
    static CmdPacket CreateFromVector(CmdType cmdType, const std::vector<uint8_t>& data);
    
    // 获取完整的数据包
    const std::vector<uint8_t>& GetPacketData() const;

    bool IsValid() const;
    
    // 获取最大包长度
    static constexpr uint16_t MAX_PACKET_SIZE = 1024;
    
private:
    // 固定头部
    static constexpr uint8_t HEADER_BYTES[4] = {0xFD, 0xFC, 0xFB, 0xFA};
    
    // 固定尾部
    static constexpr uint8_t TAIL_BYTES[4] = {0x04, 0x03, 0x02, 0x01};
    
    // 构造CmdPacket
    void ConstructPacket(const uint8_t* data, CmdType cmdType, uint16_t cmdLen);
    
    // 成员变量
    std::vector<uint8_t> packetData_;

    bool isValid_;
};

} // namespace Mmwave

#endif //__CMD_PACKET_H