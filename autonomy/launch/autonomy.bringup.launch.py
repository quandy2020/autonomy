 import argparse
import os
import sys
import subprocess


# Configuration
BINARY_PATH = "./main"  # Path to your binary
LOG_DIR = "logs"        # Directory for log files
CONFIGURATIONS = [
    {"name": "config1", "args": ["--mode=fast", "--input=data1.txt"]},
    {"name": "config2", "args": ["--mode=slow", "--input=data2.txt", "--debug"]},
    {"name": "config3", "args": ["--optimize", "--threads=4"]},
]

def launch_app(app_path):
    """启动应用程序"""
    try:
        subprocess.Popen([app_path], start_new_session=True)
        return True
    except Exception as e:
        print(f"启动失败: {e}")
        return False

def main():
    pass

if __name__ == "__main__":
    main()
        