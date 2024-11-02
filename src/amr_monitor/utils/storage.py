"""数据存储管理模块"""
from pathlib import Path
import json
import sqlite3
import pandas as pd
from threading import Thread, Lock, Event
from queue import Queue
import time
from typing import Optional, Dict, Any
from queue import Empty
import rospy

from .error_handler import handle_error, ErrorType, ErrorSeverity

class StorageManager:
    """数据存储管理器"""

    def __init__(self, config: Dict[str, Any]):
        """初始化存储管理器"""
        self.config = config
        self.base_path = Path(config.get('storage_path',
                                       '~/.ros/amr_monitor/data')).expanduser()
        self.base_path.mkdir(parents=True, exist_ok=True)

        # 数据库设置
        self.db_path = self.base_path / 'monitor.db'
        self._db_lock = Lock()
        self._write_queue = Queue(maxsize=1000)
        self._stop_event = Event()

        # 初始化数据库
        self._init_database()

        # 启动写入线程
        self._write_thread = Thread(target=self._write_worker, daemon=True)
        self._write_thread.start()

    def _init_database(self):
        """初始化数据库"""
        try:
            with sqlite3.connect(str(self.db_path)) as conn:
                conn.execute("""
                    CREATE TABLE IF NOT EXISTS monitor_data (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        timestamp REAL NOT NULL,
                        monitor_type TEXT NOT NULL,
                        data_type TEXT NOT NULL,
                        data BLOB NOT NULL,
                        metadata TEXT
                    )
                """)
                conn.commit()
        except Exception as e:
            handle_error(
                ErrorType.SYSTEM,
                ErrorSeverity.ERROR,
                f"Failed to initialize database: {e}",
                "StorageManager"
            )

    def store_data(self,
                   monitor_type: str,
                   data_type: str,
                   data: Dict[str, Any],
                   metadata: Optional[Dict[str, Any]] = None):
        """存储数据"""
        try:
            if self._write_queue.full():
                self._write_queue.get()  # 移除最旧的数据

            self._write_queue.put({
                'timestamp': time.time(),
                'monitor_type': monitor_type,
                'data_type': data_type,
                'data': json.dumps(data).encode(),
                'metadata': json.dumps(metadata) if metadata else None
            })
        except Exception as e:
            handle_error(
                ErrorType.DATA,
                ErrorSeverity.WARNING,
                f"Failed to queue data: {e}",
                "StorageManager"
            )

    def _write_worker(self):
        """数据写入工作线程"""
        while not self._stop_event.is_set():
            try:
                # 创建连接
                with sqlite3.connect(str(self.db_path)) as conn:
                    batch = []
                    while len(batch) < 100:
                        try:
                            data = self._write_queue.get(timeout=1.0)
                            batch.append(data)
                        except Empty:
                            break

                    if batch:
                        with self._db_lock:
                            conn.executemany(
                                """INSERT INTO monitor_data
                                (timestamp, monitor_type, data_type, data, metadata)
                                VALUES (?, ?, ?, ?, ?)""",
                                [(d['timestamp'], d['monitor_type'],
                                d['data_type'], d['data'], d['metadata'])
                                for d in batch]
                            )
                            conn.commit()
            except Exception as e:
                self.logger.error(f"Database error: {e}")

    def query_data(self,
                   monitor_type: str,
                   start_time: Optional[float] = None,
                   end_time: Optional[float] = None,
                   limit: Optional[int] = None) -> pd.DataFrame:
        """查询数据"""
        query = "SELECT * FROM monitor_data WHERE monitor_type = ?"
        params = [monitor_type]

        if start_time is not None:
            query += " AND timestamp >= ?"
            params.append(start_time)
        if end_time is not None:
            query += " AND timestamp <= ?"
            params.append(end_time)

        query += " ORDER BY timestamp"
        if limit:
            query += f" LIMIT {limit}"

        try:
            with self._db_lock:
                with sqlite3.connect(str(self.db_path)) as conn:
                    df = pd.read_sql_query(query, conn, params=params)
                    df['data'] = df['data'].apply(lambda x: json.loads(x.decode()))
                    if 'metadata' in df.columns:
                        df['metadata'] = df['metadata'].apply(
                            lambda x: json.loads(x) if x else None)
                    return df
        except Exception as e:
            handle_error(
                ErrorType.DATA,
                ErrorSeverity.ERROR,
                f"Data query error: {e}",
                "StorageManager"
            )
            return pd.DataFrame()

    def cleanup_old_data(self, days: int = 30):
        """清理旧数据"""
        cutoff_time = time.time() - (days * 86400)
        try:
            with self._db_lock:
                with sqlite3.connect(str(self.db_path)) as conn:
                    conn.execute(
                        "DELETE FROM monitor_data WHERE timestamp < ?",
                        (cutoff_time,)
                    )
                    conn.commit()
        except Exception as e:
            handle_error(
                ErrorType.SYSTEM,
                ErrorSeverity.WARNING,
                f"Failed to cleanup old data: {e}",
                "StorageManager"
            )

    def shutdown(self):
        """关闭存储管理器"""
        self._stop_event.set()
        if self._write_thread.is_alive():
            self._write_thread.join(timeout=5.0)