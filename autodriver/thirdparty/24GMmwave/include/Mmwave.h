#ifndef __MMWAVE_H
#define __MMWAVE_H

#include <string>
#include <CmdPacketTransmitter.h>
#include <SerialPort.h>
#include <SerialStreamProcessor.h>
#include <RangeDataProcessor.h>

namespace Mmwave {

class Mmwave
{
public:
    Mmwave(const std::string& portName, const std::string& regConfigFile);
    ~Mmwave();

    bool StartSensor(const uint32_t chirpNum = 8, uint32_t rangeBins = 8);
    void StopSensor();
    bool IsSerialGood() const { return serialPort_.isgood(); }
    bool IsDataProcessorGood() const { return rangeDataProcessor_.IsGood(); }

private:
    CmdPacketTransmitter cmdPacketTransmitter_;
    SerialPort serialPort_;
    SerialStreamProcessor serialStreamProcessor_;
    RangeDataProcessor rangeDataProcessor_;
};

}
#endif //__MMWAVE_H