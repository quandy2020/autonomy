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
import signal
import logging
import subprocess
import threading
from typing import Dict, Optional, List, Any

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from config import SystemConfig, ProcessConfig
from logger import Logger

class ProcessManager:
    """Process manager"""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.processes: Dict[str, subprocess.Popen] = {}
        self.monitor_thread: Optional[threading.Thread] = None
        self.is_monitoring = False
        self.logger = Logger.setup_logger('process_manager')
        
        # 设置信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def setup_environment(self) -> bool:
        """Setup environment variables"""
        self.logger.info("Environment setup completed (no environment file configured)")
        return True
    
    def _build_command(self, config: ProcessConfig) -> List[str]:
        """Build command line arguments based on process configuration"""
        cmd_parts = [config.binary]
        
        # 检查是否有自定义的 arguments 字段
        if hasattr(config, 'arguments') and config.arguments:
            # 处理 arguments 数组
            for arg in config.arguments:
                if isinstance(arg, dict):
                    # 处理键值对参数，如 {"-configuration_directory": "/path"}
                    for key, value in arg.items():
                        cmd_parts.append(key)
                        if value:  # 只有值非空时才添加
                            cmd_parts.append(str(value))
                elif isinstance(arg, str):
                    # 处理字符串参数
                    cmd_parts.append(arg)
                else:
                    self.logger.warning(f"Unknown argument type in {config.name}: {type(arg)}")
        
        # 向后兼容：如果没有 arguments，但存在 config_file，则使用传统方式
        elif hasattr(config, 'config_file') and config.config_file:
            self.logger.warning(f"Using legacy config_file for {config.name}, consider switching to arguments")
            if os.path.exists(config.config_file):
                cmd_parts.extend(["--config", config.config_file])
        
        self.logger.debug(f"Built command for {config.name}: {' '.join(cmd_parts)}")
        return cmd_parts
    
    def start_process(self, config_key: str, config: ProcessConfig) -> bool:
        """Start a process"""
        try:
            command = self._build_command(config)
            
            # prepare environment variables
            env = os.environ.copy()
            
            # prepare working directory
            working_dir = self.config.working_dir
            if working_dir and not os.path.exists(working_dir):
                os.makedirs(working_dir, exist_ok=True)
                self.logger.debug(f"Created working directory: {working_dir}")
            
            self.logger.debug(f"Starting {config.name} in directory: {working_dir}")
            self.logger.debug(f"Full command: {command}")
            self.logger.debug(f"Binary exists: {os.path.exists(config.binary)}")
            
            # 检查二进制文件是否存在
            if not os.path.exists(config.binary):
                self.logger.error(f"Binary file not found: {config.binary}")
                return False
            
            # 检查参数中提到的文件是否存在
            if hasattr(config, 'arguments') and config.arguments:
                for arg in config.arguments:
                    if isinstance(arg, dict):
                        for key, value in arg.items():
                            if key in ['-configuration_directory', '--config-dir', '-config']:
                                if value and not os.path.exists(value):
                                    self.logger.warning(f"Configuration directory not found: {value}")
            
            # start process
            process = subprocess.Popen(
                command,
                cwd=working_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            # 使用进程名称（从 binary 中提取的）作为键
            self.processes[config.name] = process
            
            # start output monitor
            self._start_output_monitor(config.name, process)
            
            # check startup status
            time.sleep(2)
            if process.poll() is not None:
                try:
                    stdout, stderr = process.communicate(timeout=1)
                    self.logger.error(f"Process {config.name} failed to start. Return code: {process.returncode}")
                    if stdout:
                        lines = stdout.strip().split('\n')
                        for line in lines[:5]:
                            if line.strip():
                                self.logger.error(f"Process output: {line}")
                        if len(lines) > 5:
                            self.logger.error(f"... and {len(lines) - 5} more lines")
                except subprocess.TimeoutExpired:
                    self.logger.error(f"Process {config.name} exited immediately with return code: {process.returncode}")
                
                if config.name in self.processes:
                    del self.processes[config.name]
                return False
            
            self.logger.info(f"Process {config.name} started successfully (PID: {process.pid})")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start process {config.name}: {e}")
            import traceback
            self.logger.debug(f"Stack trace: {traceback.format_exc()}")
            return False
    
    def _start_output_monitor(self, process_name: str, process: subprocess.Popen):
        """Start output monitor thread"""
        def monitor():
            try:
                while process.poll() is None:
                    output = process.stdout.readline()
                    if output:
                        cleaned_output = output.strip()
                        if cleaned_output:
                            self.logger.info(f"[{process_name}] {cleaned_output}")
            except Exception as e:
                self.logger.debug(f"Output monitor for {process_name} ended: {e}")
        
        thread = threading.Thread(target=monitor, daemon=True, name=f"OutputMonitor-{process_name}")
        thread.start()
        self.logger.debug(f"Started output monitor for {process_name}")
    
    def start_all_processes(self) -> bool:
        """Start all processes"""
        self.logger.info("Starting all processes...")
        
        success_count = 0
        config_keys = list(self.config.processes.keys())
        
        self.logger.info(f"Found {len(config_keys)} processes to start")
        
        for config_key, config in self.config.processes.items():
            
            # 记录进程的详细配置
            self.logger.debug(f"Process {config.name} configuration:")
            self.logger.debug(f"  Binary: {config.binary}")
            self.logger.debug(f"  Config key: {config_key}")
            if hasattr(config, 'arguments') and config.arguments:
                self.logger.debug(f"  Arguments: {config.arguments}")
            if hasattr(config, 'config_file') and config.config_file:
                self.logger.debug(f"  Config file: {config.config_file}")
            
            if self.start_process(config_key, config):
                success_count += 1
                self.logger.info(f"✓ Successfully started: {config.name}")
            else:
                self.logger.error(f"✗ Failed to start: {config.name}")
        
        total_processes = len(self.config.processes)
        self.logger.info(f"Started {success_count}/{total_processes} processes successfully")
        
        if success_count == total_processes:
            self.logger.info("🎉 All processes started successfully")
        elif success_count > 0:
            self.logger.warning(f"⚠️ Only {success_count}/{total_processes} processes started successfully")
        else:
            self.logger.error("💥 No processes started successfully")
            
        return success_count > 0
    
    def stop_process(self, process_name: str) -> bool:
        """Stop a process"""
        if process_name not in self.processes:
            self.logger.warning(f"Process {process_name} not found in running processes")
            return True
        
        process = self.processes[process_name]
        
        try:
            self.logger.info(f"Stopping process {process_name} (PID: {process.pid})")
            
            process.terminate()
            try:
                process.wait(timeout=5)
                self.logger.info(f"Process {process_name} terminated gracefully")
                del self.processes[process_name]
                return True
            except subprocess.TimeoutExpired:
                self.logger.warning(f"Process {process_name} did not terminate gracefully, forcing kill")
                process.kill()
                process.wait()
                del self.processes[process_name]
                self.logger.info(f"Process {process_name} killed forcefully")
                return True
                
        except Exception as e:
            self.logger.error(f"Failed to stop process {process_name}: {e}")
            return False
    
    def stop_all_processes(self):
        """Stop all processes"""
        self.logger.info("Stopping all processes...")
        
        if not self.processes:
            self.logger.info("No processes to stop")
            return
        
        for process_name in list(self.processes.keys()):
            self.stop_process(process_name)
        
        self.logger.info("All processes stopped")
    
    def _monitor_processes(self):
        """Monitor process status"""
        self.logger.debug("Process monitor thread started")
        
        while self.is_monitoring:
            try:
                for process_name, process in list(self.processes.items()):
                    returncode = process.poll()
                    if returncode is not None:
                        self.logger.warning(f"Process {process_name} (PID: {process.pid}) exited with code {returncode}")
                        
                        # 查找对应的配置
                        config = None
                        for cfg_key, cfg in self.config.processes.items():
                            if cfg.name == process_name:
                                config = cfg
                                break
                        
                        if config and config.auto_restart:
                            self.logger.info(f"Restarting process {process_name} after {config.restart_delay} seconds...")
                            time.sleep(config.restart_delay)
                            # 重新启动时需要找到对应的配置键
                            for cfg_key, cfg in self.config.processes.items():
                                if cfg.name == process_name:
                                    self.start_process(cfg_key, cfg)
                                    break
                        else:
                            del self.processes[process_name]
                            self.logger.info(f"Process {process_name} removed from monitoring (auto-restart disabled)")
                
                time.sleep(1)
            except Exception as e:
                self.logger.error(f"Error in process monitor: {e}")
                time.sleep(1)
    
    def start_monitoring(self):
        """Start monitoring process status"""
        if self.is_monitoring:
            self.logger.warning("Process monitoring is already running")
            return
            
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_processes, 
            daemon=True, 
            name="ProcessMonitor"
        )
        self.monitor_thread.start()
    
    def stop_monitoring(self):
        """Stop monitoring process status"""
        if not self.is_monitoring:
            return
            
        self.logger.info("Stopping process monitoring...")
        self.is_monitoring = False
        
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
            if self.monitor_thread.is_alive():
                self.logger.warning("Process monitor thread did not stop gracefully")
            else:
                self.logger.info("Process monitoring stopped")
    
    def get_status(self) -> Dict[str, Dict]:
        """Get process status"""
        status = {}
        for process_name, process in self.processes.items():
            returncode = process.poll()
            status[process_name] = {
                'pid': process.pid,
                'running': returncode is None,
                'returncode': returncode
            }
        return status
    
    def print_status(self):
        """Print process status"""
        status = self.get_status()
        
        if not status:
            self.logger.info("No running processes")
            return
            
        self.logger.info("Process Status Summary")
        
        for process_name, info in status.items():
            if info['running']:
                self.logger.info(f"  {process_name:20} PID {info['pid']:>6} - ✅ Running")
            else:
                self.logger.warning(f"  {process_name:20} PID {info['pid']:>6} - ❌ Exited (code: {info['returncode']})")
        
        self.logger.info(f"Total: {len(status)} process(es) running")
    
    def _signal_handler(self, signum, frame):
        """Handle signal"""
        signal_name = signal.Signals(signum).name
        self.logger.info(f"Received signal: {signal_name}")
        self.stop_monitoring()
        self.stop_all_processes()
        sys.exit(0)