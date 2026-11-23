#include <memory>
#include <iostream>
#include <vector>
#include <arpa/inet.h>
#include <RangeData.h>
#include <Checksum.h>
#include <Utils.h>

namespace Mmwave {

RangeData::RangeData() : isRangeData_(false), isSysInfo_(false)
{

}

RangeData::RangeData(const std::vector<uint8_t>& data) : isRangeData_(false), isSysInfo_(false)
{
    ParseData(data);
}

RangeData::~RangeData()
{

}

void RangeData::Dump() const
{
    if (isRangeData_) {
        std::cout << "rxChannel: " << rxChannel_;
        std::cout << " chirpIndex: " << chirpIndex_;
        std::cout << " fftNumber: " << fftNumber_ ;
        std::cout << " frameIndex: " << frameIndex_;
        std::cout << " checksum: " << checksum_ << std::endl;
    }
}

void RangeData::ParseData(const std::vector<uint8_t>& data)
{
    uint32_t firstDword, lastDword;
    std::vector<uint8_t> temp;
    Checksum<uint16_t, uint16_t> calcChecksum;

    if (data.size() > 4) {
        firstDword = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
        if (GetHeaderFromRaw(firstDword) == HEADER) {
            rxChannel_ = GetChannelFromRaw(firstDword);
            chirpIndex_ = GetChirpIndexFromRaw(firstDword);
            fftNumber_ = GetFftNumberFromRaw(firstDword) - 1; // lastDword is also included in fftData_

            if (data.size() == (fftNumber_ * 4 + 8)) {
                lastDword = data[data.size() - 4] << 24 | data[data.size() - 3] << 16 | data[data.size() - 2] << 8 | data[data.size() - 1];
                    
                if (GetFooterFromRaw(lastDword) == FOOTER) {
                    frameIndex_ = GetFrameIndexFromRaw(lastDword);
                    checksum_ = GetChecksumFromRaw(lastDword);

                    for (uint32_t i = 0; i < fftNumber_ * 2; i++) {
                        PushVector(temp, static_cast<uint16_t>(data[4 + i * 2] << 8 | data[4 + i * 2 + 1]));
                    }
                    
                    calcChecksum.update(reinterpret_cast<const uint16_t*>(temp.data()), fftNumber_ * 2);

                    if (calcChecksum.getValue() == checksum_) {
                        fftData_.clear();
                        fftData_.insert(fftData_.end(), temp.begin(), temp.end());
                        isRangeData_ = true;
                    }
                }
            }
        }
    } else if (data.size() == 4) { // sys info
        isSysInfo_ = true;
    }
}

uint32_t RangeData::GetChirpIndexFromRaw(uint32_t dword) const
{
    uint32_t chirpIndex = (dword & CHIRP_INDEX_MASK) >> CHIRP_INDEX_SHIFT;

    return ((chirpIndex & 0x00000001 << 8) | ((chirpIndex & 0x000001FE) >> 1));
}

uint32_t RangeData::GetFftNumberFromRaw(uint32_t dword) const
{
    return ((dword & FFT_NUMBER_MASK) >> FFT_NUMBER_SHIFT);
}
 
uint32_t RangeData::GetChecksumFromRaw(uint32_t dword) const
{
    return ((dword & CHECKSUM_MASK) >> CHECKSUM_SHIFT);
}

}