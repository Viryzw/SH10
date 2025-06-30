import os
import csv
import inspect
from datetime import datetime

class DataPrinter:
    def __init__(self, float_precision=2, rows=2, cols=3, log_file_name=None):
        """
        :param float_precision: 浮点数保留小数位数
        :param rows: 打印行数
        :param cols: 打印列数
        :param log_file_name: 日志文件名称；若为 None，则不记录日志
        """
        self.precision = float_precision
        self.rows = rows
        self.cols = cols
        self.log_file_name = log_file_name

    def nprint(self, *args, **kwargs):
        frame = inspect.currentframe().f_back
        all_vars = {}

        # 处理位置参数，自动推断变量名
        for value in args:
            import numpy as np
            var_names = [k for k, v in frame.f_locals.items() if np.array_equal(v, value)]

            if var_names:
                all_vars[var_names[0]] = value

        # 添加关键字参数
        all_vars.update(kwargs)
        var_names = list(all_vars.keys())

        # 日志写入（如果 log_file_name 被指定）
        if self.log_file_name:
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            write_header = not os.path.exists(self.log_file_name) or os.path.getsize(self.log_file_name) == 0

            with open(self.log_file_name, "a", newline="") as file:
                writer = csv.writer(file)
                if write_header:
                    writer.writerow(["timestamp"] + var_names)
                writer.writerow([timestamp] + [all_vars.get(name, "") for name in var_names])

        # 控制台打印
        lines = []
        for r in range(self.rows):
            row_data = []
            for c in range(self.cols):
                idx = r * self.cols + c
                if idx < len(var_names):
                    key = var_names[idx]
                    val = all_vars[key]
                    val_str = f"{val:.{self.precision}f}" if isinstance(val, float) else str(val)
                    row_data.append(f"{key} = {val_str}")
                else:
                    row_data.append("")
            lines.append("  ".join(row_data).rstrip())

        print("\n" + "\n".join(lines))

