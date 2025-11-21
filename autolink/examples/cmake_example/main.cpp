#include "autolink/common/init.hpp"
#include "autolink/common/log.hpp"
#include "autolink/time/time.hpp"

int main(int argc, char* argv[]) {
    // 初始化 autolink
    if (!autolink::Init(argv[0])) {
        AERROR << "Failed to initialize autolink";
        return 1;
    }
    
    AINFO << "Autolink initialized successfully";
    AINFO << "This is a CMake example using Autolink";
    
    // 使用 autolink 时间功能
    auto now = autolink::Time::Now();
    AINFO << "Current time: " << now.ToString();
    
    // 清理
    autolink::Clear();
    AINFO << "Autolink cleaned up";
    
    return 0;
}

