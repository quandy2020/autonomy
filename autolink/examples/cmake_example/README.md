# CMake 使用 Autolink 示例

这是一个展示如何在 CMake 项目中使用 Autolink 库的示例。

## 前置要求

1. 已安装 Autolink 库：
   ```bash
   python3 scripts/autolink.py install --prefix /usr/local
   ```

2. CMake 3.10 或更高版本

3. C++17 编译器

## 构建和运行

```bash
# 创建构建目录
mkdir build
cd build

# 配置 CMake（如果需要指定 Autolink 路径）
cmake .. -DCMAKE_PREFIX_PATH=/usr/local

# 或设置环境变量
export AUTOLINK_ROOT=/usr/local
cmake ..

# 构建
cmake --build .

# 运行
./autolink_cmake_example
```

## 文件说明

- `CMakeLists.txt` - CMake 配置文件
- `main.cpp` - 示例 C++ 代码
- `README.md` - 本文件

## 注意事项

确保 `FindAutolink.cmake` 在 CMake 可以找到的路径中。可以通过以下方式之一：

1. 将 `cmake/FindAutolink.cmake` 复制到项目的 `cmake/` 目录
2. 在 `CMakeLists.txt` 中添加路径：
   ```cmake
   list(APPEND CMAKE_MODULE_PATH "/path/to/autolink/cmake")
   ```

