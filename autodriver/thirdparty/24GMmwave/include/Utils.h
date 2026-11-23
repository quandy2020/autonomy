#ifndef __UTILS_H
#define __UTILS_H

#include <vector>
#include <cstdint>
#include <string>
#include <fstream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <iostream>

namespace Mmwave {
template<typename T>
void PushVector(std::vector<uint8_t>& destVec, const T& value)
{
    const uint8_t* valuePtr = reinterpret_cast<const uint8_t*>(&value);
    destVec.insert(destVec.end(), valuePtr, valuePtr + sizeof(T));
};

// 解析文件，每行格式为YY=XXXX，其中YY和XXXX都是16进制数字
bool ParseLine(std::ifstream& file, uint16_t& regAddr, uint16_t& value, bool& eof);

std::string GetCurrentTimeString();

std::string TimeStamp();

template<typename T>
void showHex(T value, const std::string &str)
{
    std::ios_base::fmtflags oldFlags = std::cout.flags();
    std::streamsize oldPrecision = std::cout.precision();
    char oldFill = std::cout.fill();

    std::cout << str << " = 0x" << std::hex << std::setw(sizeof(T) * 2) << std::setfill('0') << (int)value << std::endl;

    std::cout.flags(oldFlags);
    std::cout.precision(oldPrecision);
    std::cout.fill(oldFill);
}

} // namespace Mmwave

#endif //__UTILS_H