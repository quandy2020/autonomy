#ifndef __RANGE_DATA_PROCESSOR_H
#define __RANGE_DATA_PROCESSOR_H

#include <vector>
#include <cstdint>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <fstream>
#include <sstream>
#include <Chirp.h>

namespace Mmwave {

class RangeDataProcessor {
public:
    RangeDataProcessor();
    ~RangeDataProcessor();

    bool Start();
    void Stop();
    void ReceiveRangeData(const std::vector<uint8_t>& data);
    void SetConfig(uint32_t chirpNumber, uint32_t rangeBins);
    bool IsGood() const { return good_; }

private:
    enum class State
    {
        STATE_SEARCH_START_OF_FRAME = 1,
        STATE_COLLECT_RANGE_DATA = 2,
    };

    State state_;
    bool running_;

    std::queue<std::vector<uint8_t>> rangeDataQueue_;
    std::mutex rangeDataQueueMutex_;
    std::condition_variable rangeDataQueueCv_;

    std::thread processThread_;

    uint32_t chirpNumber_;
    uint32_t rangeBins_;
    uint32_t rangeDataCnt_;
    uint32_t currentFrameIndex_;
    std::vector<uint8_t> frameRangeData_;

    std::ofstream outputFile_;

    bool good_;

    Chirp chirp_;

    void ResetState();

    void ParseRangeData(const std::vector<uint8_t>& data);

    void ProcessQueue();

    void OnRangeDataReady(const std::vector<uint8_t>& data);

};

} // namespace Mmwave

#endif