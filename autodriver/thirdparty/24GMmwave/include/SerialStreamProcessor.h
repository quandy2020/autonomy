#ifndef __SERIAL_STREAM_PROCESSOR_H
#define __SERIAL_STREAM_PROCESSOR_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstdint>
#include <thread>
#include <SerialPort.h>

namespace Mmwave {

class SerialStreamProcessor
{
public:
    using StreamCallback = std::function<void(const std::vector<uint8_t>&)>;

    SerialStreamProcessor();
    ~SerialStreamProcessor();

    bool Start(StreamCallback onStreamComplete);
    void Stop();
    void ReceiveStream(const std::vector<uint8_t>& data);


private:
    static constexpr uint8_t HEADER[4] = {0x49, 0x43, 0x4C, 0x48};
    static constexpr uint8_t FOOTER[4] = {0x49, 0x43, 0x4C, 0x54};

    enum class State
    {
        STATE_SEARCH_HEADER,
        STATE_READ_LENGTH_LOW,
        STATE_READ_LENGTH_HIGH,
        STATE_READ_TYPE,
        STATE_READ_CHANNEL,
        STATE_READ_PAYLOAD,
        STATE_READ_FOOTER
    };

    State state_;
    uint16_t payloadLength_;
    uint8_t type_;
    uint8_t channel_;
    uint8_t header_index_;
    uint8_t footer_index_;

    std::vector<uint8_t> currentPayload_;

    std::queue<std::vector<uint8_t>> streamQueue_;
    std::mutex streamQueueMutex_;
    std::condition_variable streamQueueCv_;
    bool running_;
    std::thread processThread_;

    StreamCallback onStreamComplete_;

    void ResetState();

    void ParseByte(uint8_t byte);

    void ParseStream(std::vector<uint8_t>& data);

    void ProcessQueue();
    


};

} // namespace Mmwave

#endif //__STREAM_PROCESSOR_H