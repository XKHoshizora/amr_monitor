#!/bin/bash

echo "检查ROS依赖..."
ros_packages=(
    "ros-noetic-desktop-full"
    "ros-noetic-dynamic-reconfigure"
)

for pkg in "${ros_packages[@]}"; do
    if ! dpkg -l | grep -q "^ii  $pkg "; then
        echo "缺少ROS包: $pkg"
        exit 1
    fi
done

echo "检查Python依赖..."
python_packages=(
    "PyQt5"
    "numpy"
    "pandas"
    "pyqtgraph"
)

for pkg in "${python_packages[@]}"; do
    if ! python3 -c "import $pkg" 2>/dev/null; then
        echo "缺少Python包: $pkg"
        exit 1
    fi
done

echo "所有依赖检查通过"
