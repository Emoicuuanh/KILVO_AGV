"""Tiện ích dùng chung: layout, vùng, mã QR, góc."""
import math
import os

import yaml

# Thư mục gốc của package (dùng để tìm config/)
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# Đọc config/layout.yaml (bố trí nhà xưởng, vùng, mã QR)
def load_layout(path=None):
    with open(path or os.path.join(PKG, "config", "layout.yaml")) as f:
        return yaml.safe_load(f)


# Trả về tên các vùng (zones trong layout) chứa điểm (x, y)
def zones_at(layout, x, y):
    return [z["name"] for z in layout.get("zones", [])
            if z["box"][0] <= x <= z["box"][1] and z["box"][2] <= y <= z["box"][3]]


# Danh sách tọa độ mã QR: rải đều theo từng đoạn thẳng trong qr.lines, bỏ điểm trùng
def qr_positions(qr):
    pts, seen = [], set()
    for x0, y0, x1, y1, step in qr["lines"]:
        n = int(round(math.hypot(x1 - x0, y1 - y0) / step))
        for i in range(n + 1):
            t = i / n if n else 0.0
            p = (round(x0 + t * (x1 - x0), 3), round(y0 + t * (y1 - y0), 3))
            if p not in seen:
                seen.add(p)
                pts.append(p)
    return pts


# Góc yaw (rad) từ quaternion
def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# Đưa góc về khoảng [-pi, pi)
def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi
