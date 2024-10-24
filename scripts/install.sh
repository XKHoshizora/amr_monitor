#!/bin/bash

# 检查是否在工作空间中
if [ ! -f "src/amr_monitor/package.xml" ]; then
    echo "错误：请在catkin工作空间根目录运行此脚本"
    exit 1
fi

# 检查依赖
chmod +x src/amr_monitor/scripts/check_dependencies.sh
./src/amr_monitor/scripts/check_dependencies.sh
if [ $? -ne 0 ]; then
    echo "依赖检查失败，请先安装所需依赖"
    exit 1
fi

# 确保config目录存在
mkdir -p src/amr_monitor/config
mkdir -p src/amr_monitor/data

# 复制默认配置到配置文件
cp src/amr_monitor/config/default_config.yaml src/amr_monitor/config/config.yaml

# 编译功能包
catkin_make

source devel/setup.bash

# 设置执行权限
chmod +x src/amr_monitor/scripts/monitor_node.py

echo "安装完成！"
