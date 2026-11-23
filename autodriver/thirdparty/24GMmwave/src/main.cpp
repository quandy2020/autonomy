#include <thread>
#include <chrono>
#include <iostream>
#include <Mmwave.h>

int main(int argc, char** argv)
{
    if (argc != 5)
        return -1;


    while (true) {
        try {
            Mmwave::Mmwave mmwave(argv[1], argv[2]);
            mmwave.StartSensor(static_cast<uint32_t>(std::stoul(argv[3], nullptr, 10)),
                static_cast<uint32_t>(std::stoul(argv[4], nullptr, 10)));

            while (mmwave.IsSerialGood() && mmwave.IsDataProcessorGood()) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            std::cout << "serial good: " << mmwave.IsSerialGood() << ", dataprocessor good: " << mmwave.IsDataProcessorGood() << std::endl;
            if (mmwave.IsSerialGood()) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            } else {
                break;
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return 0;
        }
    }
    std::cout << "Exit" << std::endl;
    return 0; // always failed
}