#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstddef>
#include <Utils.h>
#include <iomanip>
#include <chrono>

namespace Mmwave {

bool ParseLine(std::ifstream& file, uint16_t& regAddr, uint16_t& value, bool& eof)
{   
    std::string line;

    if (!std::getline(file, line)) {
        eof = true;
        return false;
    }

    line.erase(line.find_last_not_of(" \n\r\t") + 1);
    
    std::size_t equalPos = line.find('=');
    if (equalPos == std::string::npos) {
        std::cerr << "Wrong format line: " << line << std::endl;
        return false;
    }

    std::string regAddrStr = line.substr(0, equalPos);
    std::string valueStr = line.substr(equalPos + 1);
    
    try {
        regAddr = static_cast<uint16_t>(std::stoul(regAddrStr, nullptr, 16));
    }
    catch (const std::exception& e) {
        std::cerr << "Parse reg addr failed (" << regAddrStr << "): " << e.what() << std::endl;
        return false;
    }
    
    try {
        value = static_cast<uint16_t>(std::stoul(valueStr, nullptr, 16));
    }
    catch (const std::exception& e) {
        std::cerr << "Parse value failed (" << valueStr << "): " << e.what() << std::endl;
        return false;
    }
    
    return true;
}

std::string GetCurrentTimeString()
{
    auto now = std::chrono::system_clock::now();
    auto nowTime = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&nowTime), "%Y-%m-%d_%H:%M:%S");
    return ss.str();
}

static auto start_time = std::chrono::high_resolution_clock::now();

std::string TimeStamp()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    std::stringstream ss;

    ss << duration / 1000000 << "." << std::setw(6) << std::setfill('0') << duration % 1000000;
    return ss.str();
}

} // namespace Mmwave