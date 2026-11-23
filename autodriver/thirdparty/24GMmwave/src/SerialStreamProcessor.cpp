#include <iostream>
#include <SerialStreamProcessor.h>

namespace Mmwave {

SerialStreamProcessor::SerialStreamProcessor()
    : state_(State::STATE_SEARCH_HEADER), payloadLength_(0), type_(0), channel_(0),
      header_index_(0), footer_index_(0), running_(false), processThread_() 
{

}

SerialStreamProcessor::~SerialStreamProcessor()
{
    std::cout << "~SerialStreamProcessor Begin" << std::endl;
    Stop();
    std::cout << "~SerialStreamProcessor End" << std::endl;
}

bool SerialStreamProcessor::Start(StreamCallback onStreamComplete)
{
    if (!running_) {
        running_ = true;
        onStreamComplete_ = onStreamComplete;
        processThread_ = std::thread(&SerialStreamProcessor::ProcessQueue, this);
    }

    return true;
}

void SerialStreamProcessor::Stop()
{
    if (running_) {
        running_ = false;
        streamQueueCv_.notify_one();
        if (processThread_.joinable()) {
            processThread_.join();
        }
        onStreamComplete_ = nullptr;
    }
}

void SerialStreamProcessor::ReceiveStream(const std::vector<uint8_t>& data)
{
    if (data.size()) {
        std::lock_guard<std::mutex> lock(streamQueueMutex_);
        streamQueue_.push(data);
        streamQueueCv_.notify_one();
    }
}

void SerialStreamProcessor::ResetState()
{
    state_ = State::STATE_SEARCH_HEADER;
    header_index_ = 0;
    footer_index_ = 0;
    payloadLength_ = 0;
    type_ = 0;
    channel_ = 0;
    currentPayload_.clear();
}

void SerialStreamProcessor::ParseByte(uint8_t byte)
{
    switch (state_) {
        case State::STATE_SEARCH_HEADER:
            if (byte == HEADER[header_index_]) {
                header_index_++;
                if (header_index_ == 4) {
                    header_index_ = 0;
                    state_ = State::STATE_READ_LENGTH_LOW;
                }
            } else {
                header_index_ = 0;
            }
            break;

        case State::STATE_READ_LENGTH_LOW:
            payloadLength_ = static_cast<uint16_t>(byte);
            state_ = State::STATE_READ_LENGTH_HIGH;
            break;

        case State::STATE_READ_LENGTH_HIGH:
            payloadLength_ |= static_cast<uint16_t>(byte) << 8;
            state_ = State::STATE_READ_TYPE;
            break;

        case State::STATE_READ_TYPE:
            type_ = byte;
            state_ = State::STATE_READ_CHANNEL;
            break;

        case State::STATE_READ_CHANNEL:
            channel_ = byte;
            if (payloadLength_ > 0)
                state_ = State::STATE_READ_PAYLOAD;
            else
                state_ = State::STATE_READ_FOOTER;

            break;

        case State::STATE_READ_PAYLOAD:
            currentPayload_.push_back(byte);
            if (currentPayload_.size() == payloadLength_) {
                state_ = State::STATE_READ_FOOTER;
                footer_index_ = 0;
            }
            break;
        
        case State::STATE_READ_FOOTER:
            if (byte == FOOTER[footer_index_]) {
                footer_index_++;

                if (footer_index_ == 4) {
                    if (onStreamComplete_)
                        onStreamComplete_(currentPayload_);
                    ResetState();
                }
            } else {
                ResetState();
                ParseByte(byte);
            }
            break;
        
        default:
            break;

    }
}

void SerialStreamProcessor::ParseStream(std::vector<uint8_t>& data)
{
    while (data.size()) {
        ParseByte(data.front());
        data.erase(data.begin());
    }
}

void SerialStreamProcessor::ProcessQueue()
{
    while (running_) {
        std::unique_lock<std::mutex> lock(streamQueueMutex_);
        streamQueueCv_.wait(lock, [this]() {
            return !streamQueue_.empty() || !running_;
        });

        if (!running_)
            break;

        while (!streamQueue_.empty()) {
            auto data = streamQueue_.front();
            streamQueue_.pop();
            if (data.size() == 0) break;
            lock.unlock();
            ParseStream(data);
            lock.lock();
        }
    }
}


} // namespace Mmwave