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
import time
import argparse
import logging
from pathlib import Path

# 添加当前目录到 Python 路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 导入自定义模块
from logger import Logger
from config import ConfigManager
from process import ProcessManager

def setup_logging(args):
    """设置日志配置"""
    # 根据参数配置 logger
    if args.log_file:
        # 如果指定了日志文件，创建新的 logger
        logger = Logger.setup_logger(
            'autonomy_manager', 
            args.log_level, 
            args.log_file
        )
    else:
        # 否则使用默认 logger
        logger = Logger.setup_logger('autonomy_manager', args.log_level)
    
    return logger

def main():
    parser = argparse.ArgumentParser(description='Autonomy program manager')
    parser.add_argument('--config', '-c', default='config/autonomy_config.yaml',
                       help='config file path')
    parser.add_argument('--generate-config', action='store_true',
                       help='generate example config file')
    parser.add_argument('--no-monitor', action='store_true',
                       help='do not start process monitor')
    parser.add_argument('--log-level', choices=['DEBUG', 'INFO', 'WARNING', 'ERROR'],
                       default='INFO', help='set log level')
    parser.add_argument('--log-file', help='log file path')

    # Parse arguments
    args = parser.parse_args()
    
    # 设置日志 - 这是关键修复
    logger = setup_logging(args)
    
    logger.info("Starting autonomy system...")
    logger.debug(f"Command line arguments: {sys.argv}")
    logger.debug(f"Python path: {sys.path}")
    logger.debug(f"Working directory: {os.getcwd()}")

    if args.generate_config:
        logger.info(f"Generating sample config file: {args.config}")
        try:
            ConfigManager.generate_sample_config(args.config)
            logger.info("Sample config file generated successfully")
        except Exception as e:
            logger.error(f"Failed to generate config file: {e}")
        return

    # Load configuration
    try:
        logger.info(f"Loading configuration from: {args.config}")
        config = ConfigManager.load_config(args.config)
        logger.info("Configuration loaded successfully")
        logger.debug(f"Found {len(config.processes)} processes in configuration")
        
        # 记录所有发现的进程
        for process_name in config.processes.keys():
            logger.debug(f"Discovered process: {process_name}")
            
    except FileNotFoundError:
        logger.error(f"Config file not found: {args.config}")
        logger.info("Use --generate-config to generate example config file")
        return
    except Exception as e:
        logger.error(f"Load config file failed: {e}")
        logger.info("Use --generate-config to generate example config file")
        return
        
    # Create process manager
    try:
        manager = ProcessManager(config)
        logger.debug("Process manager created successfully")
    except Exception as e:
        logger.error(f"Failed to create process manager: {e}")
        return
    
    # Start all processes
    if not manager.start_all_processes():
        logger.error("Process start failed")
        return
    
    # Start monitoring
    if not args.no_monitor:
        logger.info("Starting process monitoring...")
        manager.start_monitoring()
    
    try:
        logger.info("Autonomy system is running. Press Ctrl+C to stop.")
        
        # 主循环
        while True:
            # 定期打印状态（根据配置的间隔）
            status_interval = getattr(config, 'status_interval', 5)
            
            # 检查是否有进程异常退出
            running_processes = len(manager.get_status())
            total_processes = len(config.processes)
            
            if running_processes < total_processes:
                logger.warning(f"Some processes exited. Running: {running_processes}/{total_processes}")
            
            # 打印状态
            manager.print_status()
            
            # 等待下一次状态检查
            time.sleep(status_interval)
            
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, stopping processes...")
    except Exception as e:
        logger.error(f"Unexpected error occurred: {e}")
        logger.error("Shutting down system due to error...")
    finally:
        logger.info("Shutting down autonomy system...")
        
        # 停止监控
        if not args.no_monitor:
            manager.stop_monitoring()
            logger.info("Process monitoring stopped")
        
        # 停止所有进程
        manager.stop_all_processes()
        logger.info("All processes stopped")
        
        logger.info("Program exited successfully")


if __name__ == "__main__":
    main()