# Copyright (c) 2024 OpenRobotic Beginner Authors (duyongquan)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import yaml
from typing import Dict, Any, List
from dataclasses import dataclass, field

# 导入 logger
from logger import Logger

# 设置 logger
logger = Logger.setup_logger('config_manager')

@dataclass
class ProcessConfig:
    """Process configuration"""
    name: str  # 直接使用配置键作为进程名称
    binary: str
    arguments: List[Any] = field(default_factory=list)
    auto_restart: bool = True
    restart_delay: int = 3

@dataclass
class SystemConfig:
    """System configuration"""
    processes: Dict[str, ProcessConfig]
    log_level: str = "INFO"
    status_interval: int = 5
    working_dir: str = "./logs"

class ConfigManager:
    """Configuration manager"""
    
    @staticmethod
    def load_config(config_path: str) -> SystemConfig:
        """Load configuration file"""
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
            
            # 确保 config_data 是字典类型
            if config_data is None:
                config_data = {}
            elif not isinstance(config_data, dict):
                raise ValueError(f"Configuration file must contain a dictionary, got {type(config_data)}")
            
            logger.debug(f"Loaded config data type: {type(config_data)}")
            logger.debug(f"Config keys: {list(config_data.keys())}")
            
            # 获取根级别的配置
            shared_working_dir = config_data.get('working_dir', './logs')
            log_level = config_data.get('log_level', 'INFO')
            status_interval = config_data.get('status_interval', 5)
            
            logger.debug(f"Using shared working directory: {shared_working_dir}")
            logger.debug(f"Log level: {log_level}")
            logger.debug(f"Status interval: {status_interval}")
            
            # 解析进程配置 - 遍历所有键，找出进程配置
            processes = {}
            
            # 遍历所有配置项，找出进程配置
            for config_key, config_value in config_data.items():
                # 跳过系统配置键
                if config_key in ['log_level', 'status_interval', 'working_dir']:
                    continue
                
                if isinstance(config_value, dict) and 'binary' in config_value:
                    proc_data = config_value
                    
                    processes[config_key] = ProcessConfig(
                        name=config_key,  # 直接使用配置键作为进程名称
                        binary=proc_data.get('binary', ''),
                        arguments=proc_data.get('arguments', []),
                        auto_restart=proc_data.get('auto_restart', True),
                        restart_delay=proc_data.get('restart_delay', 3)
                    )
                    
                    logger.debug(f"Created process config for {config_key}")
                    logger.debug(f"  Binary: {proc_data.get('binary', '')}")
                    if proc_data.get('arguments'):
                        logger.debug(f"  Arguments: {proc_data['arguments']}")
            
            logger.info(f"Successfully created {len(processes)} process configurations")
            
            # 记录所有进程的映射关系
            for config_key, process_config in processes.items():
                logger.info(f"  {config_key} -> {process_config.name} ({process_config.binary})")
            
            system_config = SystemConfig(
                processes=processes,
                log_level=log_level,
                status_interval=status_interval,
                working_dir=shared_working_dir
            )
            
            return system_config
            
        except yaml.YAMLError as e:
            raise ValueError(f"YAML parsing error: {e}")
        except Exception as e:
            logger.error(f"Configuration loading error: {e}")
            raise
    
    @staticmethod
    def generate_sample_config(config_path: str):
        """Generate sample configuration file"""
        logger.info(f"Generating sample configuration file: {config_path}")
        
        sample_config = {
            'log_level': 'INFO',
            'status_interval': 5,
            'working_dir': './logs',
            'map': {
                'binary': '/workspace/autonomy/build/bin/autonomy.map.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/map"},
                    {"-configuration_basename": "map.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'bridge': {
                'binary': '/workspace/autonomy/build/bin/autonomy.bridge.launcher',
                'arguments': [
                    {"-config": "/workspace/autonomy/configuration_files/bridge/bridge.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'driver': {
                'binary': '/workspace/autonomy/build/bin/autonomy.driver.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/driver"},
                    {"-configuration_basename": "driver.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'controller': {
                'binary': '/workspace/autonomy/build/bin/autonomy.controller.launcher',
                'arguments': [
                    {"-config": "/opt/autonomy/config/controller.yaml"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'planning': {
                'binary': '/workspace/autonomy/build/bin/autonomy.planning.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/planning"},
                    {"-configuration_basename": "planning.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'perception': {
                'binary': '/workspace/autonomy/build/bin/autonomy.perception.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/perception"},
                    {"-configuration_basename": "perception.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'prediction': {
                'binary': '/workspace/autonomy/build/bin/autonomy.prediction.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/prediction"},
                    {"-configuration_basename": "prediction.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'system': {
                'binary': '/workspace/autonomy/build/bin/autonomy.system.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/system"},
                    {"-configuration_basename": "system.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'simulation': {
                'binary': '/workspace/autonomy/build/bin/autonomy.simulation.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/simulation"},
                    {"-configuration_basename": "simulation.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            },
            'visualization': {
                'binary': '/workspace/autonomy/build/bin/autonomy.visualization.launcher',
                'arguments': [
                    {"-configuration_directory": "/workspace/autonomy/configuration_files/visualization"},
                    {"-configuration_basename": "visualization.lua"}
                ],
                'auto_restart': True,
                'restart_delay': 3
            }
        }
        
        os.makedirs(os.path.dirname(config_path), exist_ok=True)
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(sample_config, f, default_flow_style=False, indent=2, allow_unicode=True)
        
        logger.info(f"Sample config file generated successfully: {config_path}")