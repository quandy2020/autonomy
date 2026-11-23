#ifndef __CHECKSUM_H
#define __CHECKSUM_H

#include <cstdint>
#include <vector>
#include <cstddef>
#include <functional>

namespace Mmwave {

template <typename T1, typename T2>
class Checksum
{
public:
    // 构造函数
    Checksum();
    
    // 重置checksum计算器
    void reset();
    
    // 更新checksum值（处理单个字节）
    void update(T2 data);
    
    // 更新checksum值（处理数据块）
    void update(const T2* data, size_t size);
    
    // 获取当前checksum值
    T1 getValue() const;
    
    // 计算数据的16位累加和checksum
    static T1 calculate(const T2* data, size_t size);
    
    // 验证16位累加和checksum
    static bool verify(const T2* data, size_t size, T1 checksum);

private:
    T1 checksumValue_;
};

template <typename T1, typename T2>
Checksum<T1, T2>::Checksum() : checksumValue_(0)
{

}

template <typename T1, typename T2>
void Checksum<T1, T2>::reset()
{
    checksumValue_ = 0;
}

template <typename T1, typename T2>
void Checksum<T1, T2>::update(T2 data)
{
    checksumValue_ += static_cast<T1>(data);
}

template <typename T1, typename T2>
void Checksum<T1, T2>::update(const T2* data, size_t size)
{
    if (data != nullptr && size > 0) {
        for (size_t i = 0; i < size; ++i) {
            update(data[i]);
        }
    }
}

template <typename T1, typename T2>
T1 Checksum<T1, T2>::getValue() const
{
    return checksumValue_;
}

template <typename T1, typename T2>
T1 Checksum<T1, T2>::calculate(const T2* data, size_t size)
{
    Checksum<T1, T2> checksum;
    checksum.update(data, size);
    return checksum.getValue();
}

template <typename T1, typename T2>
bool Checksum<T1, T2>::verify(const T2* data, size_t size, T1 checksum)
{
    return calculate(data, size) == checksum;
}

} // namespace Health

#endif // CHECKSUM_H