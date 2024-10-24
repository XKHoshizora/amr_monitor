#!/bin/bash

# 初始化缺失包的数组
missing_ros_packages=()
missing_python_packages=()

echo "检查ROS依赖..."
ros_packages=(
    "ros-noetic-desktop-full"
    "ros-noetic-dynamic-reconfigure"
)

for pkg in "${ros_packages[@]}"; do
    if ! dpkg -l | grep -q "^ii  $pkg "; then
        echo "缺少ROS包: $pkg"
        missing_ros_packages+=("$pkg")
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
        missing_python_packages+=("$pkg")
    fi
done

# 如果有缺失的包，询问是否安装
if [ ${#missing_ros_packages[@]} -ne 0 ] || [ ${#missing_python_packages[@]} -ne 0 ]; then
    echo -e "\n发现缺失的依赖包。是否要安装？ [Y/n] "
    read -r response

    # 如果用户输入Y或y或直接回车，进行安装
    if [[ "$response" =~ ^[Yy]$ ]] || [ -z "$response" ]; then
        # 安装ROS包
        if [ ${#missing_ros_packages[@]} -ne 0 ]; then
            echo "正在安装缺失的ROS包..."
            sudo apt-get update
            for pkg in "${missing_ros_packages[@]}"; do
                echo "安装 $pkg ..."
                sudo apt-get install -y "$pkg"
            done
        fi

        # 安装Python包
        if [ ${#missing_python_packages[@]} -ne 0 ]; then
            echo "正在安装缺失的Python包..."
            for pkg in "${missing_python_packages[@]}"; do
                echo "安装 $pkg ..."
                pip3 install "$pkg"
            done
        fi

        echo "所有依赖已安装完成"
        exit 0
    else
        echo "取消安装"
        exit 1
    fi
else
    echo "所有依赖检查通过"
    exit 0
fi