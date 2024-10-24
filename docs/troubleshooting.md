# AMR Monitor 故障排除指南

## 1. 启动问题

### 1.1 程序无法启动

- 检查依赖是否完整安装

  ```bash
  ./scripts/check_dependencies.sh

  ```

- 检查 ROS 环境是否正确配置

  ```bash
  printenv | grep ROS

  ```

- 检查 Python 版本是否兼容

  ```bash
  python3 --version

  ```

### 1.2 界面显示异常

- 检查 Qt 版本

  ```bash
  apt list --installed | grep qt5

  ```

- 尝试使用不同主题
- 清除配置文件重新启动

  ```bash
  rm -rf ~/.amr_monitor/*

  ```

## 2. 性能问题

### 2.1 CPU 占用高

- 降低更新频率

  ```bash
  roslaunch amr_monitor monitor.launch update_rate:=10

  ```

- 使用低资源配置

  ```bash
  cp config/low_resource.yaml ~/.amr_monitor/config.yaml

  ```

- 减小数据缓冲区大小

  ```bash
  rosparam set /amr_monitor/buffer_size 500

  ```

### 2.2 内存占用高

- 启用数据下采样
- 减少历史数据保留量
- 使用有限内存模式
- 定期清理缓存

## 3. 数据问题

### 3.1 数据显示异常

- 检查话题订阅状态

  ```bash
  rostopic list
  rostopic hz /imu/data

  ```

- 验证数据格式

  ```bash
  rostopic echo /imu/data

  ```

- 检查坐标系转换

  ```bash
  rosrun tf view_frames

  ```

### 3.2 数据记录失败

- 检查写入权限
- 验证磁盘空间
- 检查文件路径

## 4. 参数调整问题

### 4.1 参数更新失败

- 检查参数服务器连接

  ```bash
  rosparam list

  ```

- 验证参数范围
- 查看错误日志

  ```bash
  rosservice call /rosout/get_loggers

  ```

### 4.2 配置加载失败

- 验证配置文件格式
- 检查文件权限
- 使用默认配置重试

## 5. 日志级别调整

### 5.1 启用详细日志

```bash
rosservice call /amr_monitor/set_logger_level "logger: 'rosout' level: 'DEBUG'"

```

### 5.2 查看日志

```bash
rqt_console

```

## 6. 常见错误代码

### ERROR 1: 无法连接 ROS 主节点

```bash
# 解决方案
source /opt/ros/noetic/setup.bash
roscore

```

### ERROR 2: 话题订阅失败

```bash
# 检查话题
rostopic list | grep -E "imu|odom|scan"

```

### ERROR 3: 参数服务器错误

```bash
# 重置参数
rosparam delete /amr_monitor
roslaunch amr_monitor monitor.launch

```
