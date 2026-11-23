#include <cstring>
#include <algorithm>
#include <arpa/inet.h>
#include <CmdPacket.h>
#include <Utils.h>

namespace Mmwave {

// 定义静态常量
constexpr uint8_t CmdPacket::HEADER_BYTES[4];
constexpr uint8_t CmdPacket::TAIL_BYTES[4];

CmdPacket::CmdPacket()
    : isValid_(false)   
{
    packetData_.clear();
}

CmdPacket CmdPacket::CreateFromVector(CmdType cmdType, const std::vector<uint8_t>& data)
{
    CmdPacket packet;
    packet.ConstructPacket(data.data(), cmdType, data.size() + 2);
    return packet;
}

void CmdPacket::ConstructPacket(const uint8_t* data, CmdType cmdType, uint16_t cmdLen)
{
    uint16_t totalLength = sizeof(HEADER_BYTES) + 2 + cmdLen + sizeof(TAIL_BYTES);

    if (totalLength > MAX_PACKET_SIZE || cmdLen == 0) {
        return ;
    }
    
    packetData_.reserve(totalLength);
    
    packetData_.insert(packetData_.end(), std::begin(HEADER_BYTES), std::end(HEADER_BYTES));
    
    PushVector(packetData_, htole16(cmdLen));
    PushVector(packetData_, htole16(static_cast<uint16_t>(cmdType)));
    if (cmdLen > 2) {
        packetData_.insert(packetData_.end(), data, data + cmdLen - 2);
    }
    
    // 添加尾部
    packetData_.insert(packetData_.end(), std::begin(TAIL_BYTES), std::end(TAIL_BYTES));

    isValid_ = true;
}

const std::vector<uint8_t>& CmdPacket::GetPacketData() const
{
    return packetData_;
}

bool CmdPacket::IsValid() const
{
    return isValid_;
}

} // namespace Mmwave