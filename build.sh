#!/bin/bash

# Build script for Gazebo Map Creator Standalone

set -e

echo "================================"
echo "Gazebo Map Creator - Build Script"
echo "================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check for required dependencies
echo "Checking dependencies..."

check_dependency() {
    if ! command -v $1 &> /dev/null; then
        echo -e "${RED}Error: $1 is not installed${NC}"
        echo "Please install it using: sudo apt-get install $2"
        exit 1
    else
        echo -e "${GREEN}✓${NC} $1 found"
    fi
}

check_pkg_config() {
    if ! pkg-config --exists $1; then
        echo -e "${RED}Error: $1 library not found${NC}"
        echo "Please install it using: sudo apt-get install $2"
        exit 1
    else
        echo -e "${GREEN}✓${NC} $1 found"
    fi
}

check_dependency "cmake" "cmake"
check_dependency "g++" "build-essential"
check_dependency "gazebo" "gazebo11 libgazebo11-dev"
check_pkg_config "gazebo" "gazebo11 libgazebo11-dev"
# check_pkg_config "pcl_common" "libpcl-dev"
# check_pkg_config "octomap" "liboctomap-dev"

echo ""
echo "All dependencies satisfied!"
echo ""

# Create build directory
BUILD_DIR="build"
if [ -d "$BUILD_DIR" ]; then
    echo "Cleaning existing build directory..."
    rm -rf "$BUILD_DIR"
fi

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Run CMake
echo ""
echo "Running CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo ""
echo "Building project..."
make -j$(nproc)

echo ""
echo -e "${GREEN}================================${NC}"
echo -e "${GREEN}Build completed successfully!${NC}"
echo -e "${GREEN}================================${NC}"
echo ""
echo "Output files:"
echo "  - Plugin: $BUILD_DIR/libgazebo_map_creator_plugin.so"
echo "  - CLI tool: $BUILD_DIR/map_creator_cli"
echo ""
echo "To install system-wide, run:"
echo "  sudo make install"
echo ""
echo "To run the GUI application:"
echo "  python3 ../gui/map_creator_gui.py"
echo ""
