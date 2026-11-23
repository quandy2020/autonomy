#include <iostream>
#include <RangeDataProcessor.h>
#include <RangeData.h>
#include <Utils.h>

namespace Mmwave {

RangeDataProcessor::RangeDataProcessor()
    : state_(State::STATE_SEARCH_START_OF_FRAME), running_(false),
      chirpNumber_(4), rangeBins_(128), rangeDataCnt_(0), currentFrameIndex_(0), good_(true), chirp_(2)
{
    std::cout << "RangeDataProcessor Begin" << std::endl;
}

RangeDataProcessor::~RangeDataProcessor()
{
    std::cout << "~RangeDataProcessor Begin" << std::endl;
    Stop();
    std::cout << "~RangeDataProcessor End" << std::endl;
}

bool RangeDataProcessor::Start()
{
    if (!running_) {
        #if 0
        outputFile_.open("rangebin/range_data_" + GetCurrentTimeString() + ".bin", std::ios::binary);
        if (!outputFile_.is_open()) {
            std::cerr << "Failed to open output file." << std::endl;
            return false;
        }
        #endif
        
        running_ = true;
        processThread_ = std::thread(&RangeDataProcessor::ProcessQueue, this);

        return true;
    }

    return true;
}

void RangeDataProcessor::Stop()
{
    if (running_) {
        if (outputFile_.is_open()) {
            outputFile_.close();
        }

        running_ = false;
        rangeDataQueueCv_.notify_one();
        if (processThread_.joinable())
            processThread_.join();
    }
}

void RangeDataProcessor::ReceiveRangeData(const std::vector<uint8_t>& data)
{
    if (data.size()) {
        std::lock_guard<std::mutex> lock(rangeDataQueueMutex_);
        rangeDataQueue_.push(data);
        rangeDataQueueCv_.notify_one();
    }
}

void RangeDataProcessor::SetConfig(uint32_t chirpNumber, uint32_t rangeBins)
{
    chirpNumber_ = chirpNumber;
    rangeBins_ = rangeBins;
}

void RangeDataProcessor::ResetState()
{
    state_ = State::STATE_SEARCH_START_OF_FRAME;
    rangeDataCnt_ = 0;
    currentFrameIndex_ = 0;
    frameRangeData_.clear();
    chirp_.Reset();
}

void RangeDataProcessor::ParseRangeData(const std::vector<uint8_t>& data)
{
    RangeData rangeData(data);
    std::vector<uint8_t> fft_data;
    static uint32_t cnt1 = 0;

    if (!rangeData.IsRangeDate()) {
        if (!rangeData.IsSysInfo()) { // unknown subframe, sth wrong, reset state
            std::cout << "Unknown subframe received, reset RangeDataProcessor state" << std::endl;
            ResetState();
        }
        return ; // ignore sys info
    }
    fft_data = rangeData.GetFftData();
    // rangeData.Dump();

    // std::cout << "data size: " << data.size() << std::endl; 
    switch (state_) {
        case State::STATE_SEARCH_START_OF_FRAME:
            if (rangeData.IsStartOfFrame()) {
                frameRangeData_.clear();
                PushVector(frameRangeData_, 0x5A5A5A5A); // As Header

                currentFrameIndex_ = rangeData.GetFrameIndex();
                PushVector(frameRangeData_, currentFrameIndex_);

                // Total FFT data number, rangeBins_ * (rx0, rx1) * chripNumber_
                PushVector(frameRangeData_, rangeBins_ * 2 * chirpNumber_);

                // frameRangeData_.insert(frameRangeData_.end(), fft_data.begin(), fft_data.end());
                chirp_.ReceiveChannelData(rangeData.GetChannel(), fft_data.data(), fft_data.size());
                // rangeDataCnt_++;

                state_ = State::STATE_COLLECT_RANGE_DATA;
            } else {
                std::cout << "Reset State 1" << std::endl;
                cnt1++;
                ResetState();
                if (cnt1 >= (chirpNumber_ * 2))
                    good_ = false;
                
            }
            break;

        case State::STATE_COLLECT_RANGE_DATA:
            cnt1 = 0;
            if (currentFrameIndex_ == rangeData.GetFrameIndex()) {

                // frameRangeData_.insert(frameRangeData_.end(), fft_data.begin(), fft_data.end());
                chirp_.ReceiveChannelData(rangeData.GetChannel(), fft_data.data(), fft_data.size());

                if (chirp_.IsComplete()) {
                    std::vector<uint8_t> chirpData = chirp_.GetEntireData();
                    frameRangeData_.insert(frameRangeData_.end(), chirpData.begin(), chirpData.end());
                    rangeDataCnt_ += chirp_.GetNumRxChannels();
                    chirp_.Reset();
                }

                if (rangeDataCnt_ == chirpNumber_ * 2) { // Got a entire range fft data in one frame
                    PushVector(frameRangeData_, 0xA5A5A5A5); // As Footer
                    OnRangeDataReady(frameRangeData_);
                    std::cout << TimeStamp() << ": " << "Got a complete frame" << std::endl;
                    ResetState();
                }
            } else {
                std::cout << "Reset State 2, recv "<< rangeDataCnt_ << " subframes" << std::endl;
                rangeData.Dump();
                ResetState();
                ParseRangeData(data); // Maybe is start of next frame
            }
            break;

        default:
            break;
    }
}

void RangeDataProcessor::ProcessQueue()
{
    while (running_) {
        std::unique_lock<std::mutex> lock(rangeDataQueueMutex_);
        rangeDataQueueCv_.wait(lock, [this](){return !rangeDataQueue_.empty() || !running_;});

        if (!running_) {
            break;
        }

        while (!rangeDataQueue_.empty()) {
            std::vector<uint8_t> data = rangeDataQueue_.front();
            rangeDataQueue_.pop();
            lock.unlock();
            ParseRangeData(data);
            lock.lock();
        }
    }
}

void RangeDataProcessor::OnRangeDataReady(const std::vector<uint8_t>& data)
{
    if (!outputFile_.is_open())
        outputFile_.open("rangebin/range_data_" + GetCurrentTimeString() + ".bin", std::ios::binary);

    if (outputFile_.is_open()) {
        outputFile_.write(reinterpret_cast<const char*>(data.data()), data.size());
    }
} 

} // namespave Mmwave