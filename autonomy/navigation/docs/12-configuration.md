# 12 · 配置设计与模块化

## 1. 目标

配置只表达“**进程级差异**”，不复制算法默认值：

- 3 份进程配置：navigation / safety / guardian；
- 1 组平台 preset：4 种底盘；
- 1 个模型目录：DeFM/VNTPA manifest 与权重。

不再维护 `conf/plugin/*`、`conf/scenario/*`、`conf/rule/*` 预设，避免同一参数两份来源。

## 2. 目录

```text
conf/
├── navigation.pb.txt      # navigation 进程
├── safety.pb.txt          # safety 进程
├── guardian.pb.txt        # guardian 进程
├── platform/
│   ├── differential_drive.pb.txt
│   ├── omnidirectional.pb.txt
│   ├── ackermann.pb.txt
│   └── legged.pb.txt
└── models/
    ├── defm.pb.txt        # DeFM ONNX manifest
    ├── vntpa.pb.txt       # VNTPA ONNX manifest（占位）
    ├── defm_vit_s14.pth
    ├── defm_vit_s14.onnx
    └── LICENSE.defm
```

## 3. 加载关系

```text
navigation.pb.txt
  ├── platform_profile: "differential_drive"
  │     -> platform/differential_drive.pb.txt
  ├── scenario { ... }          # 内联
  ├── policy { ... }            # 内联插件参数
  ├── reflex { ... }
  ├── memory { ... }
  ├── rule { ... }
  └── perception { model_path, manifest_path }

safety.pb.txt
  ├── platform_profile
  ├── reflex { ... }
  └── chain { ... }

guardian.pb.txt
  └── heartbeat / diagnostics / allow_reflex_only

models/defm.pb.txt
  └── path/sha256/inputs/outputs
```

`LoadTextProto` 只做文本解析；`configuration_validator` 在启动前检查字段、交叉引用和模型
manifest。

## 4. 为什么删掉 plugin/scenario/rule 预设

| 旧文件 | 问题 | 现在 |
| --- | --- | --- |
| `conf/plugin/*.pb.txt` | 与 `navigation.pb.txt` 内联 policy/reflex/memory 重复 | 只保留内联 |
| `conf/scenario/*.pb.txt` | 只有名称列表，和 `scenario` 段重复 | 只保留 `scenario` 段 |
| `conf/rule/*.pb.txt` | 只有 `enabled_rules`，和 `rule` 段重复 | 只保留 `rule` 段 |

这样：

- 同一参数只有一处；
- 修改策略/规则/场景只改 `navigation.pb.txt`；
- validator 不再校验“测试夹具”，只校验实际启动配置。

## 5. 扩展点

| 新增 | 改哪里 |
| --- | --- |
| 新底盘 | `conf/platform/<name>.pb.txt` + `platform_profile` 名称 |
| 新策略 | `planning/plugins/policy/` + `navigation.pb.txt` 的 `policy.name`/options |
| 新规则 | `planning/rules/` + `navigation.pb.txt` 的 `rule.enabled_rules` |
| 新场景 | `planning/scenarios/` + `navigation.pb.txt` 的 `scenario.available_scenarios` |
| 新模型 | `conf/models/*.pb.txt` + 权重 + sha256 |

## 6. 校验

```sh
./build/navigation_configuration_validator conf
```

校验内容：

1. 三份主配置可解析；
2. 4 个平台 preset 可解析；
3. `defm.pb.txt` / `vntpa.pb.txt` 可解析；
4. 平台短名可解析；
5. 模型路径/sha256/IO 在运行时由插件再次校验。

## 7. 最小 navigation 配置

```textproto
platform_profile: "differential_drive"

scenario {
  default_scenario: "destination"
  available_scenarios: ["destination", "exploration", "following", "coverage"]
}

policy {
  name: "simulation"          # 仿真无模型；VNTPA 用 "vntpa"
}

memory { name: "pose_indexed" }
rule { enabled_rules: ["speed_limit", "safety_chain"] }

initial_goal { type: TYPE_EXPLORATION }
```

逐字段含义、消费方和调试命令见 [14-config-reference.md](14-config-reference.md)。
