#include <iostream>
#include <thread>
#include <chrono>
#include <SerialPort.h>
#include <sys/file.h>
#include <sys/ioctl.h>

namespace Mmwave {

SerialPort::SerialPort() : ioContextPtr_(nullptr), serialPortPtr_(nullptr)
{
    ioContextPtr_ = new boost::asio::io_context();
    serialPortPtr_ = new boost::asio::serial_port(*ioContextPtr_);
    good_ = true;
}

SerialPort::~SerialPort()
{
    std::cout << "~SerialPort Begin" << std::endl;
    stop_async_read();
    close();
    if (serialPortPtr_) {
        delete serialPortPtr_;
    }
    if (ioContextPtr_ && ioContextPtr_ != nullptr) {
        delete ioContextPtr_;
    }
    std::cout << "~SerialPort End" << std::endl;
}

void SerialPort::open(const std::string& portName, BaudRate baudRate, DataBits dataBits, Parity parity, 
                     StopBits stopBits, FlowControl flowControl)
{
    if (isOpen()) {
        close();
    }
    
    try {
        serialPortPtr_->open(portName);
        #if 0
        serialPortPtr_->set_option(boost::asio::serial_port_base::baud_rate(static_cast<unsigned int>(baudRate)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::character_size(convertDataBits(dataBits)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::parity(convertParity(parity)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::stop_bits(convertStopBits(stopBits)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::flow_control(convertFlowControl(flowControl)));

        portName_ = portName;
        baudRate_ = baudRate;
        dataBits_ = dataBits;
        parity_ = parity;
        stopBits_ = stopBits;
        flowControl_ = flowControl;
        #else
        (void)(baudRate);
        (void)(dataBits);
        (void)(parity);
        (void)(stopBits);
        (void)(flowControl);
        #endif
        int fd = serialPortPtr_->native_handle();
        int result = ioctl(fd, TIOCEXCL);
        if (result == -1) {
            throw std::runtime_error("Failed to set exclusive access to serial port");
        } else {
            std::cout << "Set exclusive access to serial port successfully" << std::endl;
        }
        
    } catch (const std::exception& e) {
        throw;
    }
}

void SerialPort::close()
{
    if (isOpen()) {
        try {
            if (serialPortPtr_->is_open()) {
                serialPortPtr_->close();
            }
        } catch (const std::exception& e) {
            std::cerr << "Error closing serial port: " << e.what() << std::endl;
        }
    }
}

bool SerialPort::isOpen() const
{
    return serialPortPtr_ && serialPortPtr_->is_open();
}

bool SerialPort::flush()
{
    std::vector<uint8_t> buffer;
    size_t readBytes = 0;

    if (isOpen()) {
        buffer.resize(1024);
        do {
            readBytes = read_timedwait(buffer.data(), 1024, 1000);
        } while (readBytes >= 1024);
    }
    
    return true;
}

void SerialPort::reconnect()
{
    if (isOpen()) {
        close();
    }
    
    try {
        serialPortPtr_->open(portName_);
        #if 0
        serialPortPtr_->set_option(boost::asio::serial_port_base::baud_rate(static_cast<unsigned int>(baudRate_)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::character_size(convertDataBits(dataBits_)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::parity(convertParity(parity_)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::stop_bits(convertStopBits(stopBits_)));
        serialPortPtr_->set_option(boost::asio::serial_port_base::flow_control(convertFlowControl(flowControl_)));
        #endif
        int fd = serialPortPtr_->native_handle();
        int result = ioctl(fd, TIOCEXCL);
        if (result == -1) {
            throw std::runtime_error("Failed to set exclusive access to serial port");
        } else {
            std::cout << "Set exclusive access to serial port successfully" << std::endl;
        }
    } catch (const std::exception& e) {
        throw;
    }
}

ssize_t SerialPort::write(const std::string& data)
{
    if (!isOpen()) {
        return 0;
    }
    
    try {
        return boost::asio::write(*serialPortPtr_, boost::asio::buffer(data));
    } catch (const std::exception& e) {
        std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
        return 0;
    }
}

ssize_t SerialPort::write(const uint8_t* data, size_t length)
{
    if (!isOpen() || !data || length == 0) {
        return 0;
    }
    
    try {
        return boost::asio::write(*serialPortPtr_, boost::asio::buffer(data, length));
    } catch (const std::exception& e) {
        std::cerr << "Failed to write to serial port: " << e.what() << std::endl;
        return 0;
    }
}

ssize_t SerialPort::write(const std::vector<uint8_t>& data)
{
    if (data.empty()) {
        return 0;
    }
    return write(data.data(), data.size());
}

bool SerialPort::writeByte(uint8_t byte)
{
    return write(&byte, 1) == 1;
}

ssize_t SerialPort::read(uint8_t* buffer, size_t length)
{
    if (!isOpen() || !buffer || length == 0) {
        return 0;
    }
    
    try {
        return boost::asio::read(*serialPortPtr_, boost::asio::buffer(buffer, length));
    } catch (const boost::system::system_error& e) {
        if (e.code() == boost::asio::error::misc_errors::eof) {
            return 0;
        }
        std::cerr << "Failed to read from serial port: " << e.what() << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Failed to read from serial port: " << e.what() << std::endl;
        return 0;
    }
}

bool SerialPort::readByte(uint8_t& byte)
{
    return read(&byte, 1) == 1;
}

ssize_t SerialPort::read_timedwait(uint8_t *buffer, size_t length, int timeoutMs)
{
    boost::asio::deadline_timer timer(*ioContextPtr_);
    bool timeoutOccurred = false;
    size_t bytesRead = 0;
    std::mutex mutex;
    // size_t cnt = 0;

    if (!isOpen() || !buffer || length == 0) {
        return 0;
    }

    timer.expires_from_now(boost::posix_time::milliseconds(timeoutMs));
    timer.async_wait([&](const boost::system::error_code& error) {
        std::lock_guard<std::mutex> lock(mutex);
        if (!error) {
            timeoutOccurred = true;
            serialPortPtr_->cancel();
            // std::cout << "Perform timeout callback" <<  std::endl;
        } else {
            // ioContextPtr_->stop();
            // std::cout << "Timer Error: " << error.message() << std::endl;
        }
    });

    boost::asio::async_read(*serialPortPtr_, boost::asio::buffer(buffer, length), 
        [&](const boost::system::error_code& error, size_t bytesTransfered) {
            std::lock_guard<std::mutex> lock(mutex);
            if (!error) {
                // std::cout << "Perform read callback" <<  std::endl;
                bytesRead = bytesTransfered;
                timer.cancel();
            } else {
                // ioContextPtr_->stop();
                // std::cout << "EError: " << error.message() << std::endl;
            }
        });

    // cnt = ioContextPtr_->run();
    ioContextPtr_->run();
    ioContextPtr_->reset();
    if (bytesRead == length) {
        return bytesRead;
    } else if (timeoutOccurred) {
        return 0; // timeout
    } else {
        return 0;
    }
}

ssize_t SerialPort::consumeLine(int lineNumber)
{
    ssize_t n = 0;
    boost::system::error_code ec;
    boost::asio::streambuf buffer;

    if (!isOpen())
        return 0;

    for (int i = lineNumber; i > 0; i--) {
        n += boost::asio::read_until(*serialPortPtr_, buffer, '\n');
    }
    
    return n;
}

bool SerialPort::start_async_read(std::function<void(const std::vector<uint8_t>&)> callback)
{
    if (callback == nullptr)
        return false;

    do_async_read(callback);
    asyncReadThread_ = std::thread([this]() {
        ioContextPtr_->run();
    });

    return true;
}

void SerialPort::stop_async_read()
{
    if (asyncReadThread_.joinable()) {
        ioContextPtr_->post([this]() {
            ioContextPtr_->stop();
        });
        asyncReadThread_.join();
        asyncBuffer_.clear();
    }
}

boost::asio::serial_port_base::parity::type SerialPort::convertParity(Parity parity)
{
    switch (parity) {
        case Parity::NONE: return boost::asio::serial_port_base::parity::none;
        case Parity::ODD: return boost::asio::serial_port_base::parity::odd;
        case Parity::EVEN: return boost::asio::serial_port_base::parity::even;
        default: return boost::asio::serial_port_base::parity::none;
    }
}

boost::asio::serial_port_base::stop_bits::type SerialPort::convertStopBits(StopBits stopBits)
{
    switch (stopBits) {
        case StopBits::ONE: return boost::asio::serial_port_base::stop_bits::one;
        case StopBits::TWO: return boost::asio::serial_port_base::stop_bits::two;
        default: return boost::asio::serial_port_base::stop_bits::one;
    }
}

boost::asio::serial_port_base::flow_control::type SerialPort::convertFlowControl(FlowControl flowControl)
{
    switch (flowControl) {
        case FlowControl::NONE: return boost::asio::serial_port_base::flow_control::none;
        case FlowControl::SOFTWARE: return boost::asio::serial_port_base::flow_control::software;
        case FlowControl::HARDWARE: return boost::asio::serial_port_base::flow_control::hardware;
        default: return boost::asio::serial_port_base::flow_control::none;
    }
}

boost::asio::serial_port_base::character_size SerialPort::convertDataBits(DataBits dataBits)
{
    return boost::asio::serial_port_base::character_size(static_cast<unsigned int>(dataBits));
}

void SerialPort::do_async_read(std::function<void(const std::vector<uint8_t>&)> callback)
{
    asyncBuffer_.resize(1024);

    serialPortPtr_->async_read_some(boost::asio::buffer(asyncBuffer_),
        [this, callback](const boost::system::error_code& error, size_t bytes_transferred) {
            if (!error) {
                std::vector<uint8_t> receivedData(asyncBuffer_.begin(), asyncBuffer_.begin() + bytes_transferred);
                if (callback)
                    callback(receivedData);
                do_async_read(callback);
            } else {
                if (error == boost::asio::error::operation_aborted)
                    std::cout << "Async read operation aborted." << std::endl;
                else
                    std::cerr << "Async read failed: " << error.message() << std::endl;
                try {
                    reconnect();
                    do_async_read(callback);
                } catch (const std::exception& e) {
                    // should terminate process
                    good_ = false;
                }
            }
        });
}

} // namespave Health
