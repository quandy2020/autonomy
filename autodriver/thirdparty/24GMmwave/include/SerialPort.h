#ifndef __SERIAL_PORT_H
#define __SERIAL_PORT_H

#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>
#include <functional>
#include <thread>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

namespace Mmwave {

enum class BaudRate : uint32_t
{
    BAUD_9600 = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
    BAUD_57600 = 57600,
    BAUD_115200 = 115200,
    BAUD_921600 = 921600,
};

enum class DataBits
{
    FIVE = 5,
    SIX = 6,
    SEVEN = 7,
    EIGHT = 8
};

enum class Parity
{
    NONE,
    ODD,
    EVEN
};

enum class StopBits
{
    ONE,
    TWO
};

enum class FlowControl
{
    NONE,
    SOFTWARE,
    HARDWARE
};

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();

    void open(const std::string& portName, BaudRate baudRate, DataBits dataBits, Parity parity, StopBits stopBits, FlowControl flowControl);
    void close();
    bool isOpen() const;
    bool flush();
    void reconnect();
    bool isgood() const { return good_; }

    ssize_t write(const std::string& data);
    ssize_t write(const uint8_t* data, size_t length);
    ssize_t write(const std::vector<uint8_t>& data);
    bool writeByte(uint8_t byte);

    ssize_t read(uint8_t* buffer, size_t maxLength);
    bool readByte(uint8_t& byte);
    ssize_t read_timedwait(uint8_t *buffer, size_t maxLength, int timeoutMs);

    ssize_t consumeLine(int lineNumber);

    bool start_async_read(std::function<void(const std::vector<uint8_t>&)> callback);
    void stop_async_read();

private:
    boost::asio::io_context* ioContextPtr_;
    boost::asio::serial_port* serialPortPtr_;
    std::thread asyncReadThread_;
    std::vector<uint8_t> asyncBuffer_;

    std::string portName_;
    BaudRate baudRate_;
    DataBits dataBits_;
    Parity parity_;
    StopBits stopBits_;
    FlowControl flowControl_;
    bool good_;
    
    void initializeSerialPort();
    boost::asio::serial_port_base::parity::type convertParity(Parity parity);
    boost::asio::serial_port_base::stop_bits::type convertStopBits(StopBits stopBits);
    boost::asio::serial_port_base::flow_control::type convertFlowControl(FlowControl flowControl);
    boost::asio::serial_port_base::character_size convertDataBits(DataBits dataBits);

    void do_async_read(std::function<void(const std::vector<uint8_t>&)> callback);
};

} // namespace Mmwave

#endif // SERIAL_PORT_H
