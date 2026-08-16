# automsgs RPC CLI（grpcurl 封装）

日期：2026-08-14  
状态：已批准（方案 A）  
范围：`automsgs/tools/cli/rpc-cli.py`

## 目标

Python 薄封装 `grpcurl`，对 `automsgs/proto/rpcs` 做 `list` / `describe` / `call`，不生成 gRPC stub；覆盖全部域 Service。

## 决策

- 源文件：`rpc-cli.py`；安装名：`rpc-cli`  
- 通用三命令，无按域子命令  
- import 布局与 CMake `proto_include/automsgs/{msgs,rpcs,...}` 一致（本地用 symlink）  
- 默认 plaintext + `localhost:50051`  
- 依赖系统已安装 `grpcurl`
