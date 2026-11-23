#include <Chirp.h>

namespace Mmwave {
Chirp::Chirp(uint32_t numRxChannels) : numRxChannels_(numRxChannels)
{
    channelBuffers_.reserve(numRxChannels);
    channelBuffersReady_.assign(numRxChannels, false);
}

bool Chirp::ReceiveChannelData(uint32_t channel, const uint8_t *data, uint32_t numBytes)
{
    if (channel >= numRxChannels_) {
        return false;
    }

    if (channelBuffersReady_[channel]) {
        return false;
    }

    channelBuffers_[channel].reserve(numBytes);

    channelBuffers_[channel].assign(data, data + numBytes);
    channelBuffersReady_[channel] = true;
    return true;
}

bool Chirp::IsComplete() const
{
    for (uint32_t i = 0; i < numRxChannels_; i++) {
        if (!channelBuffersReady_[i]) {
            return false;
        }
    }
    return true;
}

void Chirp::Reset(void)
{
    std::fill(channelBuffersReady_.begin(), channelBuffersReady_.end(), false);
    channelBuffers_.clear();
}

Chirp::~Chirp()
{
    Reset();
}

std::vector<uint8_t> Chirp::GetEntireData(void) const
{
    std::vector<uint8_t> data;

    if (!IsComplete()) {
        return std::vector<uint8_t>();
    }

    for (uint32_t i = 0; i < numRxChannels_; i++) {
        data.insert(data.end(), channelBuffers_[i].begin(), channelBuffers_[i].end());
    }

    return data;
}

}