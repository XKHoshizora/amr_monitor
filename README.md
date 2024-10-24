# AMR Monitor

AMR Monitor 是一个用于监控和调试自主移动机器人(AMR)的ROS功能包。它提供了直观的图形界面，用于实时显示和记录机器人的传感器数据，以及动态调整导航相关参数。

![AMR Monitor Screenshot](docs/images/screenshot.png)

## ✨ 特性

### 📊 数据监控
- IMU数据实时显示（角速度、线加速度）
- 里程计数据可视化（位置、速度）
- 激光雷达数据2D显示
- 运动指令(cmd_vel)监控

### ⚙️ 参数调整
- AMCL参数实时调整
- DWA参数动态配置
- 代价地图参数优化

### 💾 数据管理
- CSV格式数据导出
- 数据回放功能
- 自动时间戳命名

### 🎨 界面定制
- 暗色/亮色主题切换
- 自定义布局支持
- 多种可视化选项

### 📈 数据分析
- 基础统计分析
- 频谱分析
- 相关性分析
- 异常检测

## 🚀 快速开始

### 环境要求
- ROS Noetic (Ubuntu 20.04)
- Python 3.x
- PyQt5
- numpy, pandas, pyqtgraph

### 安装步骤

1. 克隆仓库：
```bash
cd ~/catkin_ws/src
git clone https://github.com/XKHoshizora/amr_monitor.git
```

2. 安装依赖：
```bash
sudo apt-get update
sudo apt-get install python3-pyqt5 python3-pyqt5.qtchart python3-numpy python3-pandas ros-noetic-dynamic-reconfigure python3-pyqtgraph
```

3. 编译功能包：
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 运行

1. 启动监控器：
```bash
roslaunch amr_monitor monitor.launch
```

2. 查看数据：
   - 左侧窗口显示实时传感器数据图表
   - 支持鼠标缩放和平移查看详细数据

3. 调整参数：
   - 右侧窗口包含可调整的导航参数
   - 使用数值框直接输入或微调

4. 记录数据：
   - 点击"开始记录"按钮开始数据记录
   - 数据自动保存为CSV格式

## 📖 详细文档

### 配置说明

1. 话题配置
```yaml
topics:
  imu: /imu/data
  odom: /odom
  scan: /scan
  cmd_vel: /cmd_vel
```

2. 性能配置
```xml
<launch>
    <node name="amr_monitor" pkg="amr_monitor" type="monitor_node.py" output="screen">
        <param name="update_rate" value="30" />  <!-- 更新频率(Hz) -->
        <param name="buffer_size" value="1000" /> <!-- 数据缓冲大小 -->
    </node>
</launch>
```

### 使用技巧

1. 数据分析：
   - 使用内置分析工具研究数据特征
   - 支持数据导出进行离线分析
   - 提供异常检测功能

2. 布局定制：
   - 拖拽组件自定义布局
   - 保存布局配置
   - 支持多显示器

3. 性能优化：
   - 使用多线程模式提升性能
   - 优化数据缓存减少内存占用
   - 支持30Hz的更新频率

## 🤝 贡献

欢迎提交问题和改进建议！提交PR时请：
1. Fork本仓库
2. 创建功能分支
3. 提交更改
4. 推送到分支
5. 创建Pull Request

## 📝 许可证

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件

## 👥 作者

XKHoshizora - hoshizoranihon@gmail.com

## ✨ 致谢

- ROS社区
- PyQt团队
- 所有贡献者

## 📊 更新日志

### v1.0.0
- 初始版本发布
- 基本监控功能
- 参数调整
- 数据记录

### v1.1.0
- 添加数据回放
- 支持主题切换
- 优化性能
- 添加分析工具

## Star History

[![Star History Chart](https://api.star-history.com/svg?repos=XKHoshizora/amr_monitor&type=Date)](https://star-history.com/#XKHoshizora/amr_monitor&Date)