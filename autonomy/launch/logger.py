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
import logging
from typing import Optional

class Logger:
    """日志管理类"""
    
    @staticmethod
    def setup_logger(
        name: str = 'autonomy_manager',
        log_level: str = 'INFO',
        log_file: Optional[str] = None,
        console_format: str = '[0;32m[%(asctime)s][0m %(levelname)s: %(message)s',
        file_format: str = '[%(asctime)s] %(levelname)s: %(message)s',
        datefmt: str = '%Y-%m-%d %H:%M:%S'
    ) -> logging.Logger:
        """
        设置并返回配置好的 logger
        
        Args:
            name: logger 名称
            log_level: 日志级别 ('DEBUG', 'INFO', 'WARNING', 'ERROR')
            log_file: 日志文件路径，如果为 None 则不写入文件
            console_format: 控制台输出格式
            file_format: 文件输出格式
            datefmt: 日期格式
        
        Returns:
            logging.Logger: 配置好的 logger 实例
        """
        logger = logging.getLogger(name)
        
        # 避免重复配置
        if logger.handlers:
            return logger
        
        # 设置日志级别
        level = getattr(logging, log_level.upper(), logging.INFO)
        logger.setLevel(level)
        
        # 控制台 handler（带颜色）
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_formatter = logging.Formatter(console_format, datefmt=datefmt)
        console_handler.setFormatter(console_formatter)
        logger.addHandler(console_handler)
        
        # 文件 handler（不带颜色）
        if log_file:
            # 确保日志目录存在
            log_dir = os.path.dirname(log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir, exist_ok=True)
                
            file_handler = logging.FileHandler(log_file, encoding='utf-8')
            file_handler.setLevel(logging.DEBUG)  # 文件记录所有级别的日志
            file_formatter = logging.Formatter(file_format, datefmt=datefmt)
            file_handler.setFormatter(file_formatter)
            logger.addHandler(file_handler)
        
        return logger
    
    @staticmethod
    def get_logger(name: str = 'autonomy_manager') -> logging.Logger:
        """
        获取已配置的 logger
        
        Args:
            name: logger 名称
            
        Returns:
            logging.Logger: logger 实例
        """
        return logging.getLogger(name)
    
    @staticmethod
    def set_level(logger: logging.Logger, level: str):
        """
        设置日志级别
        
        Args:
            logger: logger 实例
            level: 日志级别 ('DEBUG', 'INFO', 'WARNING', 'ERROR')
        """
        log_level = getattr(logging, level.upper(), logging.INFO)
        logger.setLevel(log_level)
        for handler in logger.handlers:
            handler.setLevel(log_level)