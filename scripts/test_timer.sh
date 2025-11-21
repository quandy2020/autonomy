#!/bin/bash
# Timer 测试脚本

set -e

echo "=========================================="
echo "  Timer 构建和测试"
echo "=========================================="

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查是否在 Docker 容器中
if [ -f /.dockerenv ]; then
    WORKSPACE_DIR="/workspace/autonomy"
    IN_DOCKER=true
else
    WORKSPACE_DIR="/Users/quandy/Workspace/project/autonomy"
    IN_DOCKER=false
fi

BUILD_DIR="${WORKSPACE_DIR}/build"

echo -e "${YELLOW}工作目录: ${WORKSPACE_DIR}${NC}"
echo -e "${YELLOW}构建目录: ${BUILD_DIR}${NC}"
echo ""

# 步骤 1: 构建
echo -e "${GREEN}[1/3] 构建项目...${NC}"
cd ${BUILD_DIR}
ninja

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 构建成功${NC}"
else
    echo -e "${RED}✗ 构建失败${NC}"
    exit 1
fi

echo ""

# 步骤 2: 运行测试
echo -e "${GREEN}[2/3] 运行 Timer 测试...${NC}"
ctest -R timer_test --output-on-failure

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 测试通过${NC}"
else
    echo -e "${RED}✗ 测试失败${NC}"
    exit 1
fi

echo ""

# 步骤 3: 显示测试统计
echo -e "${GREEN}[3/3] 测试统计...${NC}"
ctest -R timer_test -N

echo ""
echo -e "${GREEN}=========================================="
echo -e "  所有测试完成！"
echo -e "==========================================${NC}"

exit 0

