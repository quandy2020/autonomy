#!/bin/bash
# Setup script for running autolink examples

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Set AUTOLINK_PATH to point to the autolink source directory
export AUTOLINK_PATH="${SCRIPT_DIR}/src/autonomy/autolink/autolink"

# Export the path for easy access
export PATH="${SCRIPT_DIR}/src/autonomy/build/bin:${PATH}"

echo "AUTOLINK_PATH=${AUTOLINK_PATH}"
echo "Running: $@"
exec "$@"

