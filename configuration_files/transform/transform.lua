-- Copyright 2025 The Openbot Authors(duyongquan)
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Transform Server 配置
-- =====================
-- 该配置文件定义了TF系统的核心参数

AUTONOMY_TRANSFORM = {
    -- 静态TF配置文件路径（YAML格式）
    -- 包含所有静态坐标系变换的定义
    static_transform_config = "configuration_files/transform/static_transform.yaml",
    
    -- TF缓冲区大小（秒）
    -- 定义TF系统保存历史变换的时间长度
    buffer_cache_time = 10.0,
    
    -- TF查询默认超时时间（秒）
    -- 当查询TF变换时的最大等待时间
    default_timeout = 0.01,
    
    -- 调试模式
    -- 启用后会输出更详细的TF调试信息
    debug = false,
    
    -- TF发布频率（Hz）
    -- 静态TF的发布频率
    publish_rate = 10.0,
}

return AUTONOMY_TRANSFORM
