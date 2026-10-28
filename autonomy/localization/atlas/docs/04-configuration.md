# 04 配置

参考配置位于 `config/`：

- `euroc_mono.yaml`：EuRoC 单目。
- `euroc_mono_inertial.yaml`：EuRoC 单目惯性。
- `euroc_stereo_inertial.yaml`：EuRoC 双目惯性。
- `tum_vi_stereo_inertial.yaml`：TUM-VI 双目惯性。
- `tum_rgbd.yaml`：TUM RGB-D。
- `tartanair_rgbd_inertial.yaml`：TartanAir RGB-D 惯性。
- `orb_vocab.fbow`：二进制 FBow 词袋。

验证状态：EuRoC 单目惯性和双目惯性配置均已通过真实数据集验证；
TUM-VI 双目惯性仍待真实数据集验证；TartanAir RGB-D 惯性目前仅通过
接口和合成数据测试，仍待完整真实数据集验证。配置和离线示例可用不等同于
精度已经验证。

ConfigLoader 读取相机内参、畸变、图像尺寸、ORB 参数、双目基线、IMU
噪声以及系统加载/保存选项。配置中的相机模型必须与输入图像一致：
Pinhole 用于针孔模型，Kannala-Brandt 用于鱼眼模型。

配置调整原则：

1. 时间戳统一使用数据集或消息提供的原始时钟，图像与 IMU 必须处于同一
   时间基准。
2. 双目左右相机应提供一致的分辨率和正确的外参。
3. IMU 噪声和随机游走参数必须来自标定，不应通过轨迹结果反向调参。
4. ORB 特征数量和金字塔层数会直接影响 CPU、内存和弱纹理场景表现。
5. 修改配置后至少运行一条普通序列和一条困难序列。

安装时 YAML 与 FBow 文件会复制到
`share/autonomy/localization/conf/atlas`。
