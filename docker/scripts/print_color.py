#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The OpenRobotic Beginner Authors (duyongquan). All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""
Color logging utilities for better output formatting.
Provides colored logging functions using Python's logging module.
"""

import logging
import sys


class ColoredFormatter(logging.Formatter):
    """Custom formatter that adds colors to log messages."""

    # ANSI color codes
    COLORS = {
        'DEBUG': '\033[36m',      # Cyan
        'INFO': '\033[92m',       # Green
        'WARNING': '\033[93m',    # Yellow
        'ERROR': '\033[91m',      # Red
        'CRITICAL': '\033[95m',   # Magenta
    }
    RESET = '\033[0m'

    def format(self, record):
        """Format log record with color."""
        # Get the color for this log level
        color = self.COLORS.get(record.levelname, '')
        
        # Format the message
        log_message = super().format(record)
        
        # Add color if output is a terminal
        if sys.stdout.isatty() and color:
            log_message = f"{color}{log_message}{self.RESET}"
        
        return log_message


def setup_logger(name: str = __name__, level: int = logging.INFO) -> logging.Logger:
    """
    Setup and configure a colored logger.
    
    Args:
        name: Logger name (default: module name)
        level: Logging level (default: INFO)
    
    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    
    # Avoid adding multiple handlers if logger already configured
    if logger.handlers:
        return logger
    
    logger.setLevel(level)
    
    # Create console handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(level)
    
    # Create colored formatter
    formatter = ColoredFormatter(
        fmt='[%(levelname)s] %(message)s',
        datefmt=None
    )
    handler.setFormatter(formatter)
    
    # Add handler to logger
    logger.addHandler(handler)
    
    return logger


# Create default logger instance
_logger = setup_logger('autonomy_docker', logging.INFO)


def print_error(message: str) -> None:
    """
    Print an error message in red using logger.
    
    Args:
        message: Error message to print
    """
    _logger.error(message)


def print_warning(message: str) -> None:
    """
    Print a warning message in yellow using logger.
    
    Args:
        message: Warning message to print
    """
    _logger.warning(message)


def print_info(message: str) -> None:
    """
    Print an info message in green using logger.
    
    Args:
        message: Info message to print
    """
    _logger.info(message)


def print_debug(message: str) -> None:
    """
    Print a debug message in cyan using logger.
    
    Args:
        message: Debug message to print
    """
    _logger.debug(message)


def get_logger(name: str = None, level: int = logging.INFO) -> logging.Logger:
    """
    Get a logger instance for custom use.
    
    Args:
        name: Logger name (default: None, uses default logger)
        level: Logging level (default: INFO)
    
    Returns:
        Logger instance
    """
    if name:
        return setup_logger(name, level)
    return _logger


# For backward compatibility with old print_color function
def print_color(color_code: str, message: str) -> None:
    """
    Print a colored message (backward compatibility).
    
    Args:
        color_code: ANSI color code (ignored, uses level-based colors)
        message: Message to print
    """
    _logger.info(message)
