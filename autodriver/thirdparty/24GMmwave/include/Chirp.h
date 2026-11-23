#ifndef __CHIRP_H
#define __CHIRP_H

#include <vector>
#include <cstdint>

namespace Mmwave{
class Chirp {
public:
    Chirp(uint32_t numRxChannels);
    Chirp(const Chirp &) = delete;
    Chirp& operator=(const Chirp &) = delete;
    ~Chirp();
    bool ReceiveChannelData(uint32_t channel, const uint8_t *data, uint32_t numBytes);
    bool IsComplete(void) const;
    void Reset(void);
    std::vector<uint8_t> GetEntireData(void) const;
    uint32_t GetNumRxChannels(void) const { return numRxChannels_; }

private:
    uint32_t numRxChannels_;
    std::vector<std::vector<uint8_t>> channelBuffers_;
    std::vector<bool> channelBuffersReady_;

};

} // namespace Mmwave

#endif // __CHIRP_H