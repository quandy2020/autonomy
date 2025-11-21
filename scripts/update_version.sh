#!/bin/bash
# Version Update Script for Autonomy Project
# Usage: ./update_version.sh <major> <minor> <patch>

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VERSION_FILE="${PROJECT_ROOT}/version.json"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to display usage
usage() {
    echo "Usage: $0 <major> <minor> <patch>"
    echo ""
    echo "Examples:"
    echo "  $0 1 0 0        # Update to version 1.0.0"
    echo "  $0 0 1 0        # Update to version 0.1.0"
    echo "  $0 0 0 4        # Update to version 0.0.4"
    exit 1
}

# Check arguments
if [ $# -ne 3 ]; then
    echo -e "${RED}Error: Invalid number of arguments${NC}"
    usage
fi

MAJOR=$1
MINOR=$2
PATCH=$3
VERSION="${MAJOR}.${MINOR}.${PATCH}"
DATE=$(date +%Y-%m-%d)

# Validate input (must be numbers)
if ! [[ "$MAJOR" =~ ^[0-9]+$ ]] || ! [[ "$MINOR" =~ ^[0-9]+$ ]] || ! [[ "$PATCH" =~ ^[0-9]+$ ]]; then
    echo -e "${RED}Error: Version numbers must be integers${NC}"
    usage
fi

# Display current version
if [ -f "$VERSION_FILE" ]; then
    CURRENT_VERSION=$(grep -o '"version"[^"]*"[^"]*"' "$VERSION_FILE" | sed 's/"version"[^"]*"//;s/"//')
    echo -e "${YELLOW}Current version: ${CURRENT_VERSION}${NC}"
fi

# Confirm update
echo -e "${GREEN}New version: ${VERSION}${NC}"
read -p "Update version? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Version update cancelled."
    exit 0
fi

# Update version.json
cat > "$VERSION_FILE" << JSON
{
  "version": "${VERSION}",
  "major": ${MAJOR},
  "minor": ${MINOR},
  "patch": ${PATCH},
  "name": "Autonomy",
  "description": "Autonomous Robot Development Framework",
  "build_date": "${DATE}"
}
JSON

echo -e "${GREEN}✓ Version updated to ${VERSION}${NC}"
echo -e "${YELLOW}📝 Updated: ${VERSION_FILE}${NC}"
echo ""
echo "Next steps:"
echo "  1. cd build && cmake .."
echo "  2. make -j\$(nproc)"
echo "  3. git add version.json"
echo "  4. git commit -m \"chore: bump version to ${VERSION}\""

