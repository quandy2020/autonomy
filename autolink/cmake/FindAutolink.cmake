# FindAutolink.cmake
# 查找 Autolink 库的 CMake 模块
#
# 使用方法：
#   find_package(Autolink REQUIRED)
#   target_link_libraries(your_target PRIVATE Autolink::Autolink)
#
# 变量：
#   Autolink_FOUND          - 是否找到 Autolink
#   Autolink_INCLUDE_DIRS    - Autolink 头文件目录
#   Autolink_LIBRARIES       - Autolink 库文件
#   Autolink_VERSION         - Autolink 版本
#
# 导入目标：
#   Autolink::Autolink       - Autolink 库目标

include(FindPackageHandleStandardArgs)

# 设置默认搜索路径
set(_Autolink_ROOT_PATHS
    ${CMAKE_INSTALL_PREFIX}
    ${CMAKE_PREFIX_PATH}
    $ENV{AUTOLINK_ROOT}
    $ENV{HOME}/.local
    /usr/local
    /opt/local
    /usr
)

# 查找头文件
find_path(Autolink_INCLUDE_DIR
    NAMES autolink/common/init.hpp
    PATHS ${_Autolink_ROOT_PATHS}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)

# 如果没找到，使用默认路径搜索
if(NOT Autolink_INCLUDE_DIR)
    find_path(Autolink_INCLUDE_DIR
        NAMES autolink/common/init.hpp
        PATHS ${_Autolink_ROOT_PATHS}
        PATH_SUFFIXES include
    )
endif()

# 查找库文件（优先查找共享库）
find_library(Autolink_SHARED_LIBRARY
    NAMES autolink
    PATHS ${_Autolink_ROOT_PATHS}
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
)

find_library(Autolink_STATIC_LIBRARY
    NAMES autolink
    PATHS ${_Autolink_ROOT_PATHS}
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
)

# 如果没找到，使用默认路径搜索
if(NOT Autolink_SHARED_LIBRARY AND NOT Autolink_STATIC_LIBRARY)
    find_library(Autolink_SHARED_LIBRARY
        NAMES autolink
        PATHS ${_Autolink_ROOT_PATHS}
        PATH_SUFFIXES lib lib64
    )
    
    find_library(Autolink_STATIC_LIBRARY
        NAMES autolink
        PATHS ${_Autolink_ROOT_PATHS}
        PATH_SUFFIXES lib lib64
    )
endif()

# 优先使用共享库
if(Autolink_SHARED_LIBRARY)
    set(Autolink_LIBRARY ${Autolink_SHARED_LIBRARY})
elseif(Autolink_STATIC_LIBRARY)
    set(Autolink_LIBRARY ${Autolink_STATIC_LIBRARY})
endif()

# 设置包含目录
if(Autolink_INCLUDE_DIR)
    set(Autolink_INCLUDE_DIRS ${Autolink_INCLUDE_DIR})
endif()

# 设置库文件
if(Autolink_LIBRARY)
    set(Autolink_LIBRARIES ${Autolink_LIBRARY})
endif()

# 查找版本信息（如果存在）
if(EXISTS "${Autolink_INCLUDE_DIR}/autolink/version.hpp")
    file(READ "${Autolink_INCLUDE_DIR}/autolink/version.hpp" _version_file)
    string(REGEX MATCH "AUTOLINK_VERSION_MAJOR[ \t]+([0-9]+)" _major_match "${_version_file}")
    string(REGEX MATCH "AUTOLINK_VERSION_MINOR[ \t]+([0-9]+)" _minor_match "${_version_file}")
    if(_major_match AND _minor_match)
        set(Autolink_VERSION_MAJOR ${CMAKE_MATCH_1})
        set(Autolink_VERSION_MINOR ${CMAKE_MATCH_2})
        set(Autolink_VERSION "${Autolink_VERSION_MAJOR}.${Autolink_VERSION_MINOR}")
    endif()
else()
    set(Autolink_VERSION "0.1.0")
endif()

# 检查是否找到所有必需的组件
find_package_handle_standard_args(Autolink
    REQUIRED_VARS Autolink_LIBRARY Autolink_INCLUDE_DIR
    VERSION_VAR Autolink_VERSION
)

# 如果找到了，创建导入目标
if(Autolink_FOUND AND NOT TARGET Autolink::Autolink)
    if(Autolink_SHARED_LIBRARY)
        add_library(Autolink::Autolink SHARED IMPORTED)
    else()
        add_library(Autolink::Autolink STATIC IMPORTED)
    endif()
    
    set_target_properties(Autolink::Autolink PROPERTIES
        IMPORTED_LOCATION "${Autolink_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${Autolink_INCLUDE_DIRS}"
    )
    
    # 设置依赖库（如果需要）
    # 例如：protobuf, glog 等
    # find_package(Protobuf REQUIRED)
    # set_target_properties(Autolink::Autolink PROPERTIES
    #     INTERFACE_LINK_LIBRARIES "protobuf::libprotobuf"
    # )
endif()

# 打印找到的信息（用于调试）
if(Autolink_FOUND AND Autolink_FIND_DEBUG)
    message(STATUS "Autolink found:")
    message(STATUS "  Version: ${Autolink_VERSION}")
    message(STATUS "  Include dir: ${Autolink_INCLUDE_DIRS}")
    message(STATUS "  Library: ${Autolink_LIBRARIES}")
endif()

