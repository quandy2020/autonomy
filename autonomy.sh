#!/usr/bin/env bash

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

# Autonomy Process Manager
# Usage: 
#   ./autonomy.sh start    - 启动所有进程
#   ./autonomy.sh stop     - 停止所有进程
#   ./autonomy.sh status   - 查看进程状态
#   ./autonomy.sh restart  - 重启所有进程
#   ./autonomy.sh logs     - 查看日志


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Configuration File
CONFIG_FILE="configuration_files/autonomy.yaml"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="logs"
LOG_FILE="${LOG_DIR}/autonomy_${TIMESTAMP}.log"
PID_FILE="${LOG_DIR}/autonomy.pid"  # 使用固定名称，不使用时间戳

# Color Definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# =============================================================================
# Function Definitions
# =============================================================================

# Logging Functions
function log() 
{
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a "$LOG_FILE"
}

function error() 
{
    echo -e "${RED}[$(date '+%Y-%m-%d %H:%M:%S')] ERROR:${NC} $1" | tee -a "$LOG_FILE"
}

function warn() 
{
    echo -e "${YELLOW}[$(date '+%Y-%m-%d %H:%M:%S')] WARN:${NC} $1" | tee -a "$LOG_FILE"
}

function info() 
{
    echo -e "${BLUE}[$(date '+%Y-%m-%d %H:%M:%S')] INFO:${NC} $1" | tee -a "$LOG_FILE"
}

# Dependency Check Functions
function check_dependencies () 
{
    if ! command -v python3 &> /dev/null; then
        error "python3 not found. Please install python3."
        return 1
    fi
    
    if ! python3 -c "import yaml" &> /dev/null; then
        error "Python yaml module not found. Please install pyyaml using: pip3 install pyyaml"
        return 1
    fi
    
    return 0
}

# Directory Creation Functions
function create_directories() 
{
    mkdir -p "$LOG_DIR"
    mkdir -p "$(dirname "$CONFIG_FILE")"
}

# Configuration File Check Functions
function check_config() 
{
    if [[ ! -f "$CONFIG_FILE" ]]; then
        warn "Configuration file not found: $CONFIG_FILE"
        info "Generating example configuration file..."
        python3 autonomy/launch/bringup.py --generate-config "$CONFIG_FILE"
        if [[ $? -ne 0 ]]; then
            error "Failed to generate configuration file. Please check permissions and try again."
            return 1
        fi
    fi
    return 0
}

# PID File Management Functions
function save_pid() 
{
    echo $1 > "$PID_FILE"
}

function get_pid() 
{
    if [[ -f "$PID_FILE" ]]; then
        cat "$PID_FILE" 2>/dev/null
    else
        echo ""
    fi
}

# Process Status Check Functions
function is_running() 
{
    local pid=$(get_pid)
    if [[ -z "$pid" ]]; then
        return 1
    fi
    
    if kill -0 "$pid" 2>/dev/null; then
        return 0
    else
        # Invalid PID found, remove PID file
        warn "Invalid PID found in $PID_FILE. Cleaning up..."
        rm -f "$PID_FILE"
        return 1
    fi
}

# Enhanced process checking
function is_process_running() 
{
    local pid=$1
    if [[ -z "$pid" ]]; then
        return 1
    fi
    
    # Check if process exists and is our autonomy process
    if ps -p "$pid" > /dev/null 2>&1; then
        # Check if it's actually our autonomy process
        local process_name=$(ps -p "$pid" -o comm= 2>/dev/null)
        if [[ "$process_name" == *"python"* ]]; then
            local cmd_line=$(ps -p "$pid" -o command= 2>/dev/null)
            if [[ "$cmd_line" == *"bringup.py"* ]]; then
                return 0
            fi
        fi
    fi
    return 1
}

# Process Status Check Functions
function check_process_status() 
{
    local proc_name=$1
    local display_name=$2
    
    local pids=$(pgrep -f "$proc_name" 2>/dev/null)
    if [[ -n "$pids" ]]; then
        echo -e "${GREEN}✓ $display_name运行中 (PID: ${pids//$'\n'/, })${NC}"
    else
        echo -e "${RED}✗ $display_name未运行${NC}"
    fi
}

# Process Management Functions
function stop_related_processes() 
{
    log "Killing all autonomy.* processes..."
    
    # 直接使用 pkill 强制杀死所有 autonomy 进程
    pkill -9 -f "autonomy\." 2>/dev/null
    
    # 等待一下让进程完全终止
    sleep 2
    
    # 检查是否还有残留进程
    local remaining=$(pgrep -f "autonomy\." 2>/dev/null)
    if [[ -n "$remaining" ]]; then
        warn "Some autonomy processes still running, retrying..."
        pkill -9 -f "autonomy\." 2>/dev/null
        sleep 1
    fi
    
    local final_check=$(pgrep -f "autonomy\." 2>/dev/null)
    if [[ -n "$final_check" ]]; then
        error "Failed to kill autonomy processes: $final_check"
        return 1
    else
        log "All autonomy.* processes killed"
        return 0
    fi
}

# Enhanced stop function
function stop_autonomy_manager() 
{
    local pid=$(get_pid)
    
    if [[ -z "$pid" ]]; then
        warn "No PID file found, searching for autonomy manager process..."
        # 尝试通过进程名查找
        pid=$(pgrep -f "bringup.py" | head -1)
        if [[ -z "$pid" ]]; then
            log "No autonomy manager process found"
            return 0
        fi
    fi
    
    if is_process_running "$pid"; then
        log "Stopping autonomy manager (PID: $pid)..."
        
        # 发送 TERM 信号
        kill -TERM "$pid" 2>/dev/null
        
        # 等待进程结束
        local count=0
        while is_process_running "$pid" && [[ $count -lt 10 ]]; do
            sleep 1
            ((count++))
            echo -n "."
        done
        echo
        
        if is_process_running "$pid"; then
            warn "Process $pid did not respond to TERM, sending KILL..."
            kill -KILL "$pid" 2>/dev/null
            sleep 2
        fi
        
        if is_process_running "$pid"; then
            error "Failed to stop process $pid"
            return 1
        else
            log "Autonomy manager stopped successfully"
            # 清理 PID 文件
            if [[ -f "$PID_FILE" ]]; then
                rm -f "$PID_FILE"
            fi
            return 0
        fi
    else
        warn "Autonomy manager process $pid is not running"
        # 清理无效的 PID 文件
        if [[ -f "$PID_FILE" ]]; then
            rm -f "$PID_FILE"
        fi
        return 0
    fi
}

# =============================================================================
# Main Function Definitions
# =============================================================================

# Start Function
function start_autonomy() 
{
    # Pre-checks
    if ! check_dependencies; then
        return 1
    fi
    
    create_directories
    
    if ! check_config; then
        return 1
    fi
    
    # Check if system is already running
    if is_running; then
        warn "Autonomy manager is already running in process (PID: $(get_pid))"
        return 1
    fi
    
    # 清理可能存在的旧 PID 文件
    if [[ -f "$PID_FILE" ]]; then
        rm -f "$PID_FILE"
    fi
    
    log "Starting autonomy system..."
    log "Config file: $CONFIG_FILE"
    log "Log file: $LOG_FILE"
    
    # Start autonomy manager (background)
    nohup python3 autonomy/launch/bringup.py --config "$CONFIG_FILE" >> "$LOG_FILE" 2>&1 &
    local pid=$!
    
    # Save PID
    save_pid $pid
    
    # Wait for startup and verify
    sleep 5
    
    if is_process_running "$pid"; then
        log "Autonomy manager started successfully! (PID: $pid)"
        log "Use './autonomy.sh status' to check status"
        log "Use './autonomy.sh stop' to stop the system"
        return 0
    else
        error "Autonomy manager failed to start. Check log file: $LOG_FILE"
        # 检查进程退出状态
        if wait "$pid" 2>/dev/null; then
            error "Process exited with code: $?"
        fi
        # 清理无效的 PID 文件
        if [[ -f "$PID_FILE" ]]; then
            rm -f "$PID_FILE"
        fi
        return 1
    fi
}

# Stop Function
function stop_autonomy() 
{
    log "Stopping autonomy system..."
    
    # 首先停止管理器
    if ! stop_autonomy_manager; then
        error "Failed to stop autonomy manager"
        return 1
    fi
    
    # 然后停止相关进程
    if ! stop_related_processes; then
        error "Failed to stop some autonomy processes"
        return 1
    fi
    
    log "Autonomy system stopped successfully."
    return 0
}

# Restart Function
function restart_autonomy() 
{
    log "Restarting autonomy system..."
    
    if stop_autonomy; then
        sleep 3
        if start_autonomy; then
            log "Autonomy system restarted successfully."
            return 0
        else
            error "Failed to start autonomy system after stop."
            return 1
        fi
    else
        error "Failed to stop autonomy system for restart."
        return 1
    fi
}

# Status Function
function show_status() 
{
    echo -e "${BLUE}=== Autonomy System Status ===${NC}"
    
    local manager_pid=$(get_pid)
    if is_process_running "$manager_pid"; then
        echo -e "${GREEN}✓ 管理器运行中 (PID: $manager_pid)${NC}"
        
        # 显示 autonomy 相关进程
        local autonomy_pids=$(pgrep -f "autonomy\." 2>/dev/null)
        if [[ -n "$autonomy_pids" ]]; then
            echo -e "${GREEN}✓ Autonomy 进程运行中:${NC}"
            for pid in $autonomy_pids; do
                local process_info=$(ps -p "$pid" -o pid,comm,command --no-headers 2>/dev/null)
                if [[ -n "$process_info" ]]; then
                    echo "  - $process_info"
                fi
            done
        else
            echo -e "${YELLOW}⚠ 没有找到运行的 autonomy 进程${NC}"
        fi
        
        echo
        echo -e "Log File: $LOG_FILE"
        if [[ -n "$manager_pid" ]]; then
            echo -e "Start Time: $(ps -p "$manager_pid" -o lstart= 2>/dev/null || echo 'Unknown')"
        fi
        
    else
        echo -e "${RED}✗ 管理器未运行${NC}"
        
        # 检查是否有遗留的 autonomy 进程
        local stray_pids=$(pgrep -f "autonomy\." 2>/dev/null)
        if [[ -n "$stray_pids" ]]; then
            echo
            warn "发现遗留的 autonomy 进程:"
            for pid in $stray_pids; do
                local process_info=$(ps -p "$pid" -o pid,comm,command --no-headers 2>/dev/null)
                echo "  $process_info"
            done
            echo
            echo -e "使用 './autonomy.sh stop' 清理遗留进程"
        fi
        
        # 清理无效的 PID 文件
        if [[ -f "$PID_FILE" ]]; then
            warn "发现无效的 PID 文件，正在清理..."
            rm -f "$PID_FILE"
        fi
    fi
}

# Logs Function
function show_logs() 
{
    if [[ ! -d "$LOG_DIR" ]]; then
        error "Log directory not found: $LOG_DIR"
        return 1
    fi
    
    echo -e "${BLUE}=== Log Files ===${NC}"
    ls -lh "$LOG_DIR/" 2>/dev/null || {
        error "No log files found"
        return 1
    }
    
    return 0
}

# Clean Function
function clean_logs() 
{
    log "Cleaning log files..."
    
    if [[ -d "$LOG_DIR" ]]; then
        rm -rf "$LOG_DIR"
        log "Log directory cleaned"
    else
        log "Log directory does not exist"
    fi
    return 0
}

# Help Function
function show_help() 
{
    echo -e "${BLUE}Autonomy Process Manager${NC}"
    echo
    echo "Usage: $0 {start|stop|restart|status|logs|clean|help}"
    echo
    echo "Commands:"
    echo -e "  ${GREEN}start${NC}   - 启动自主系统"
    echo -e "  ${GREEN}stop${NC}    - 停止自主系统"
    echo -e "  ${GREEN}restart${NC} - 重启自主系统"
    echo -e "  ${GREEN}status${NC}  - 显示系统状态"
    echo -e "  ${GREEN}logs${NC}    - 显示日志文件"
    echo -e "  ${GREEN}clean${NC}   - 清理日志文件"
    echo -e "  ${GREEN}help${NC}    - 显示帮助信息"
    echo
    echo "Examples:"
    echo "  $0 start"
    echo "  $0 status"
    echo "  $0 stop"
}

# =============================================================================
# Main Function
# =============================================================================

function main() 
{
    local command="${1:-}"
    
    case "$command" in
        start)
            if start_autonomy; then
                exit 0
            else
                exit 1
            fi
            ;;
        stop)
            if stop_autonomy; then
                exit 0
            else
                exit 1
            fi
            ;;
        restart)
            if restart_autonomy; then
                exit 0
            else
                exit 1
            fi
            ;;
        status)
            show_status
            exit 0
            ;;
        logs)
            if show_logs; then
                exit 0
            else
                exit 1
            fi
            ;;
        clean)
            if clean_logs; then
                exit 0
            else
                exit 1
            fi
            ;;
        help|--help|-h)
            show_help
            exit 0
            ;;
        "")
            error "请指定一个命令"
            show_help
            exit 1
            ;;
        *)
            error "未知命令: $command"
            show_help
            exit 1
            ;;
    esac
}

# Call Main Function
main "$@"