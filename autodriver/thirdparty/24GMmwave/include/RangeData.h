#ifndef __RANGE_DATA_H
#define __RANGE_DATA_H

#include <vector>
#include <cstdint>

namespace Mmwave {

class RangeData {
public:
    RangeData();
    RangeData(const std::vector<uint8_t>& data);
    ~RangeData();

    bool IsRangeDate() const { return isRangeData_; }
    bool IsSysInfo() const { return isSysInfo_; }
    bool IsStartOfFrame() const { return ((chirpIndex_ == 0)); };

    uint32_t GetChannel() const { return rxChannel_; }
    uint32_t GetChirpIndex() const { return chirpIndex_; }
    uint32_t GetFftNumber() const { return fftNumber_; }
    uint32_t GetChecksum() const { return checksum_; }
    uint32_t GetFrameIndex() const { return frameIndex_; }
    const std::vector<uint8_t> GetFftData() const { return fftData_; }

    void Dump() const;

private:
    static constexpr uint8_t HEADER = 0xaa;
    static constexpr uint8_t FOOTER = 0x55;
    static constexpr uint8_t HEADER_SHIFT = 24;
    static constexpr uint32_t HEARDER_MASK = (0xff << HEADER_SHIFT);
    static constexpr uint8_t CHANNEL_SHIFT = 23;
    static constexpr uint32_t CHANNEL_MASK = (0x1 << CHANNEL_SHIFT);
    static constexpr uint8_t CHIRP_INDEX_SHIFT = 11;
    static constexpr uint32_t CHIRP_INDEX_MASK = (0x1ff << CHIRP_INDEX_SHIFT);
    static constexpr uint8_t FFT_NUMBER_SHIFT = 0;
    static constexpr uint32_t FFT_NUMBER_MASK = (0x7ff << FFT_NUMBER_SHIFT);
    static constexpr uint8_t CHECKSUM_SHIFT = 16;
    static constexpr uint32_t CHECKSUM_MASK = (0xffff << CHECKSUM_SHIFT);
    static constexpr uint8_t FRAME_INDEX_SHIFT = 12;
    static constexpr uint32_t FRAME_INDEX_MASK = (0xf << FRAME_INDEX_SHIFT);
    static constexpr uint8_t FOOTER_SHIFT = 0;
    static constexpr uint32_t FOOTER_MASK = (0xff << FOOTER_SHIFT);

    uint32_t rxChannel_;
    uint32_t chirpIndex_;
    uint32_t fftNumber_;
    std::vector<uint8_t> fftData_; // host byte seq
    uint32_t checksum_;
    uint32_t frameIndex_;
    std::vector<uint8_t> rawData_;

    bool isRangeData_;
    bool isSysInfo_;

    void ParseData(const std::vector<uint8_t>& data);
    uint32_t GetHeaderFromRaw(uint32_t dword) const {return ((dword & HEARDER_MASK) >> HEADER_SHIFT); }
    uint32_t GetChannelFromRaw(uint32_t dword) const { return ((dword & CHANNEL_MASK) >> CHANNEL_SHIFT); }
    uint32_t GetChirpIndexFromRaw(uint32_t dword) const;
    uint32_t GetFftNumberFromRaw(uint32_t dword) const;
    uint32_t GetChecksumFromRaw(uint32_t dword) const;
    uint32_t GetFrameIndexFromRaw(uint32_t dword) const { return ((dword & FRAME_INDEX_MASK) >> FRAME_INDEX_SHIFT); }
    uint32_t GetFooterFromRaw(uint32_t dword) const { return ((dword & FOOTER_MASK) >> FOOTER_SHIFT); }
};

} // namespace Mmwave

#endif