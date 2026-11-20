set(CROSS_CPU_FLAGS "-march=armv8-a")
# A matching host CUDA cross toolkit must be supplied before enabling CUDA.
set(CMAKE_CUDA_ARCHITECTURES 87 CACHE STRING "Jetson Orin CUDA architecture")
