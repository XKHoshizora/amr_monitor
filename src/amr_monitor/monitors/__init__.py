import sys
from pathlib import Path

# 获取项目根目录路径
project_root = Path(__file__).resolve().parents[3]

# 将 src 目录添加到 sys.path 中
src_path = project_root / "src"
sys.path.append(str(src_path))
