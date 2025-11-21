"""Autolink Bazel 宏函数

提供简化的宏函数来创建依赖 autolink 库的二进制文件和测试。
"""

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_test", "cc_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")

def autolink_proto(name = "proto", proto_files = None, examples_proto_files = None, deps = None, **kwargs):
    """
    创建 autolink proto 库及其 C++ 代码生成目标，自动包含所有 proto 文件。
    
    用法:
        autolink_proto(
            proto_files = glob(["proto/*.proto"]),
            examples_proto_files = glob(["examples/proto/*.proto"]),
        )
        # 自动创建：
        #   - proto_library (name = "proto")
        #   - cc_proto_library (name = "cc_proto")
        #   - cc_library (name = "proto_cc")
        #   - proto_library (name = "examples_proto")
        #   - cc_proto_library (name = "examples_cc_proto")
        #   - cc_library (name = "examples_proto_cc")
    
    参数:
        name: proto 库名称（默认 "proto"）
        proto_files: proto 文件列表（可选），如果不指定则自动发现 proto/*.proto
        examples_proto_files: examples proto 文件列表（可选），如果不指定则自动发现 examples/proto/*.proto
        deps: 额外的依赖（可选），会自动添加 @protobuf//:api_proto
        **kwargs: 传递给 proto_library 的其他参数
    """
    if deps == None:
        deps = []
    
    # 如果没有指定 proto_files，自动发现
    if proto_files == None:
        proto_files = native.glob([
            "proto/*.proto",
        ])
    
    # 如果没有指定 examples_proto_files，自动发现
    if examples_proto_files == None:
        examples_proto_files = native.glob([
            "examples/proto/*.proto",
        ])
    
    # 自动添加 protobuf 依赖
    proto_deps = deps + [
        "@protobuf//:api_proto",
    ]
    
    # 创建主 proto 库
    # 设置 strip_import_prefix 以便 proto 文件可以使用 autolink/ 前缀导入
    proto_library(
        name = name,
        srcs = proto_files,
        deps = proto_deps,
        strip_import_prefix = "/",
        **kwargs
    )
    
    # 创建 C++ proto 代码生成库
    cc_proto_library(
        name = "cc_proto",
        deps = [":" + name],
    )
    
    # 创建 C++ proto 包装库
    # includes 设置为 "." 以便包含 autolink/ 前缀的路径可以正确解析
    native.cc_library(
        name = "proto_cc",
        deps = [
            ":cc_proto",
            "@protobuf//:protobuf",
        ],
        includes = ["."],
    )
    
    # 创建 examples proto 库（如果存在 examples proto 文件）
    if examples_proto_files:
        proto_library(
            name = "examples_proto",
            srcs = examples_proto_files,
            strip_import_prefix = "/",
        )
        
        cc_proto_library(
            name = "examples_cc_proto",
            deps = [":examples_proto"],
        )
        
        native.cc_library(
            name = "examples_proto_cc",
            deps = [
                ":examples_cc_proto",
                "@protobuf//:protobuf",
            ],
            includes = ["."],
        )

def autolink_library(name = "autolink", srcs = None, hdrs = None, asms = None, deps = None, **kwargs):
    """
    创建 autolink 统一库，自动包含所有子模块的源文件和头文件。
    
    用法:
        autolink_library(
            srcs = glob(["**/*.cpp"]),
            hdrs = glob(["**/*.hpp"]),
        )
        # 创建名为 "autolink" 的库，使用指定的源文件和头文件
        # 自动处理平台特定的汇编文件（aarch64 和 x86_64）
    
    参数:
        name: 库名称（默认 "autolink"）
        srcs: 源文件列表（可选），如果不指定则自动发现所有 .cpp 文件（排除测试和示例）
        hdrs: 头文件列表（可选），如果不指定则自动发现所有 .hpp 文件（排除测试和示例）
        asms: 平台特定的汇编文件（select 结果，可选），如果不指定则自动使用默认的平台特定汇编文件
        deps: 额外的依赖（可选），会自动添加 :proto_cc, @fastdds//:fastdds, @glog//:glog, @nlohmann_json//:json, @protobuf//:protobuf
        **kwargs: 传递给 cc_library 的其他参数
    """
    if deps == None:
        deps = []
    
    # 如果没有指定 srcs，使用默认的 glob
    if srcs == None:
        srcs = native.glob([
            "**/*.cpp",
        ], exclude = [
            "**/*_test.cpp",
            "**/test/**",
            "**/example/**",
            "examples/**",
        ])
    
    # 如果没有指定 hdrs，使用默认的 glob
    if hdrs == None:
        hdrs = native.glob([
            "**/*.hpp",
        ], exclude = [
            "**/*_test.hpp",
            "**/test/**",
            "**/example/**",
            "examples/**",
        ])
    
    # 如果没有指定 asms，自动使用平台特定的汇编文件
    if asms == None:
        asms = select({
            "@platforms//cpu:aarch64": ["croutine/detail/swap_aarch64.S"],
            "@platforms//cpu:x86_64": ["croutine/detail/swap_x86_64.S"],
            "//conditions:default": [],
        })
    
    # 自动添加 autolink 库的依赖
    deps = deps + [
        ":proto_cc",
        "@fastdds//:fastdds",
        "@glog//:glog",
        "@nlohmann_json//:json",
        "@protobuf//:protobuf",
    ]
    
    native.cc_library(
        name = name,
        srcs = srcs + asms,
        hdrs = hdrs,
        deps = deps,
        **kwargs
    )

def autolink_binary(name, srcs, deps = None, **kwargs):
    """
    创建一个依赖 autolink 库的二进制文件。
    
    用法:
        autolink_binary(
            name = "talker",
            srcs = ["examples/talker.cpp"],
        )
    
    参数:
        name: 二进制文件名称
        srcs: 源文件列表
        deps: 额外的依赖（可选），会自动添加 :autolink 和 :examples_proto_cc
        **kwargs: 传递给 cc_binary 的其他参数
    """
    if deps == None:
        deps = []
    
    # 自动添加 autolink 库和 examples_proto_cc 依赖
    deps = deps + [
        ":autolink",
        ":examples_proto_cc",
    ]
    
    cc_binary(
        name = name,
        srcs = srcs,
        deps = deps,
        **kwargs
    )

def autolink_test(name = None, srcs = None, deps = None, **kwargs):
    """
    自动创建测试目标，根据测试文件自动生成。
    
    用法1 - 自动从文件名推导名称:
        autolink_test(
            srcs = ["base/atomic_hash_map_test.cpp"],
        )
        # 自动创建名为 "atomic_hash_map_test" 的测试目标
    
    用法2 - 明确指定名称:
        autolink_test(
            name = "my_test",
            srcs = ["my_test.cpp"],
        )
    
    参数:
        name: 测试目标名称（可选），如果不指定则从 srcs[0] 文件名推导
        srcs: 测试源文件列表（必需）
        deps: 额外的依赖（可选），会自动添加 :autolink, @googletest//:gtest, @googletest//:gtest_main
        **kwargs: 传递给 cc_test 的其他参数
    """
    if srcs == None or len(srcs) == 0:
        fail("autolink_test: srcs must be specified")
    
    # 如果没有指定 name，从 srcs[0] 推导
    if name == None:
        src_file = srcs[0]
        # 移除路径和扩展名
        if src_file.endswith("_test.cpp"):
            name = src_file[:-9].split("/")[-1] + "_test"  # 保留 "_test" 后缀
        elif src_file.endswith(".cpp"):
            name = src_file[:-4].split("/")[-1]
        else:
            fail("autolink_test: cannot derive test name from srcs, please specify name explicitly")
    
    if deps == None:
        deps = []
    
    # 自动添加 autolink 库和测试框架依赖
    deps = deps + [
        ":autolink",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ]
    
    cc_test(
        name = name,
        srcs = srcs,
        deps = deps,
        includes = ["."],  # Allow includes from package root
        **kwargs
    )

def autolink_tests(test_files, deps = None, test_deps_map = None, test_data_map = None, **kwargs):
    """
    批量创建多个测试目标。
    
    用法:
        autolink_tests(
            test_files = glob(["**/*_test.cpp"]),
        )
        # 为每个测试文件自动创建测试目标
        
        # 为特定测试指定额外依赖和数据文件
        autolink_tests(
            test_files = glob(["**/*_test.cpp"]),
            test_deps_map = {
                "data/data_visitor_test.cpp": ["@protobuf//:protobuf"],
                "logger/async_logger_test.cpp": ["@glog//:glog"],
            },
            test_data_map = {
                "transport/transport_shm_talker_test.cpp": ["conf/autolink.pb.conf"],
            },
        )
    
    参数:
        test_files: 测试文件列表（通常使用 glob() 获取）
        deps: 额外的依赖（可选），会自动添加到所有测试
        test_deps_map: 测试文件到额外依赖的映射（可选），格式为 {test_file: [deps]}
        test_data_map: 测试文件到数据文件的映射（可选），格式为 {test_file: [data_files]}
        **kwargs: 传递给每个 cc_test 的其他参数
    """
    if test_deps_map == None:
        test_deps_map = {}
    if test_data_map == None:
        test_data_map = {}
    
    for test_file in test_files:
        # 从文件名推导测试名称
        if test_file.endswith("_test.cpp"):
            test_name = test_file[:-9].split("/")[-1] + "_test"
        elif test_file.endswith(".cpp"):
            test_name = test_file[:-4].split("/")[-1]
        else:
            continue  # 跳过不符合命名规范的文件
        
        # 合并通用依赖和特定测试的依赖
        test_specific_deps = test_deps_map.get(test_file, [])
        combined_deps = (deps or []) + test_specific_deps
        
        # 获取特定测试的数据文件
        test_specific_data = test_data_map.get(test_file, [])
        
        # 创建测试，如果有数据文件则添加 data 参数
        test_kwargs = dict(kwargs)
        if test_specific_data:
            test_kwargs["data"] = test_specific_data
        
        autolink_test(
            name = test_name,
            srcs = [test_file],
            deps = combined_deps if combined_deps else None,
            **test_kwargs
        )

