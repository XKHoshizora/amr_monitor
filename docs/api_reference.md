# AMR Monitor API 参考

## 1. 核心类

### 1.1 MainWindow

主窗口类，管理整体 UI 和功能协调。

### 方法

```python
def update_data(self):
    """更新显示数据"""
    pass

def toggle_recording(self, checked: bool):
    """控制数据记录"""
    pass

def export_data(self):
    """导出数据到文件"""
    pass

```

### 1.2 PlotWidget

绘图组件类，处理数据可视化。

### 方法

```python
def update_plots(self):
    """更新所有图表"""
    pass

def setup_data_buffers(self):
    """初始化数据缓冲区"""
    pass

def imu_callback(self, msg: Imu):
    """处理IMU数据回调"""
    pass

```

### 1.3 ParameterWidget

参数控制组件类，管理参数调整。

### 方法

```python
def update_amcl_param(self, name: str, value: float):
    """更新AMCL参数"""
    pass

def refresh_parameters(self):
    """刷新所有参数"""
    pass

```

## 2. 工具类

### 2.1 DataProcessor

数据处理工具类。

### 方法

```python
def process_imu_data(self, msg: Imu) -> dict:
    """处理IMU数据"""
    pass

def process_odom_data(self, msg: Odometry) -> dict:
    """处理里程计数据"""
    pass

```

### 2.2 ConfigManager

配置管理工具类。

### 方法

```python
def load_config(self) -> dict:
    """加载配置"""
    pass

def save_config(self, config: dict):
    """保存配置"""
    pass

```

## 3. 数据结构

### 3.1 数据缓冲区

```python
class DataBuffer:
    """
    数据缓冲区结构

    属性:
        size (int): 缓冲区大小
        buffer (np.ndarray): 数据数组
        index (int): 当前索引
    """
    pass

```

### 3.2 参数配置

```python
class Parameter:
    """
    参数配置结构

    属性:
        name (str): 参数名称
        value (float): 参数值
        min_val (float): 最小值
        max_val (float): 最大值
        step (float): 步进值
    """
    pass

```

## 4. 回调接口

### 4.1 传感器数据回调

```python
def sensor_callback(msg):
    """
    传感器数据回调接口

    参数:
        msg: ROS消息对象
    """
    pass

```

### 4.2 参数更新回调

```python
def parameter_callback(config):
    """
    参数更新回调接口

    参数:
        config: 参数配置字典
    """
    pass

```

## 5. 扩展接口

### 5.1 添加新的数据类型

```python
def add_data_type(name, topic_type, processor):
    """
    添加新的数据类型

    参数:
        name: 数据类型名称
        topic_type: ROS消息类型
        processor: 数据处理器
    """
    pass

```

### 5.2 添加新的分析工具

```python
def add_analysis_tool(name, analyzer):
    """
    添加新的分析工具

    参数:
        name: 工具名称
        analyzer: 分析器对象
    """
    pass
```
