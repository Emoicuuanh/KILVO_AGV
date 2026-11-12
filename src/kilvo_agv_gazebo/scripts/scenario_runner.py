#!/usr/bin/env python3
"""Chạy kịch bản thực nghiệm (config/scenario.yaml): điều khiển AGV vòng kín theo ground truth
để quỹ đạo lặp lại y hệt giữa các lần chạy; xuất nhãn vùng.

Ra: /cmd_vel, /sim/zone (String), /sim/step (String), /sim/scenario_done (Bool, latched)
"""
import math
import os

import rospy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from agv_common import PKG, load_layout, wrap, yaw_from_quat, zones_at


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class ScenarioRunner:
    DT = 0.02

    def __init__(self):
        sf = rospy.get_param("~scenario_file", "") or os.path.join(PKG, "config", "scenario.yaml")
        with open(sf) as f:
            self.sc = yaml.safe_load(f)
        self.layout = load_layout(rospy.get_param("~layout_file", "") or None)
        self.steps = self.sc["steps"]
        self.v_def = self.sc.get("default_speed", 1.0)
        self.acc = self.sc.get("accel", 0.6)
        self.pub_cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_zone = rospy.Publisher("/sim/zone", String, queue_size=10)
        self.pub_step = rospy.Publisher("/sim/step", String, queue_size=10, latch=True)
        self.pub_done = rospy.Publisher("/sim/scenario_done", Bool, queue_size=1, latch=True)
        self.pose = None
        self.idx = -1
        self.st = {}
        self.v_cmd = self.w_cmd = 0.0
        self.prev_xy = None
        self.t_start = None
        self.done = False
        rospy.Subscriber("/ground_truth/odom", Odometry, self.cb_gt, queue_size=50)
        rospy.Timer(rospy.Duration(self.DT), self.tick)
        rospy.Timer(rospy.Duration(0.1), self.pub_zones)
        rospy.loginfo("Kịch bản %s: %d bước", sf, len(self.steps))

    def cb_gt(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def pub_zones(self, _):
        if self.pose:
            self.pub_zone.publish(String(data=",".join(zones_at(self.layout, *self.pose[:2]))))

    def send(self, v, w):
        self.v_cmd, self.w_cmd = v, w
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub_cmd.publish(t)

    def ramp(self, v_target):
        dv = self.acc * self.DT
        return clamp(v_target, self.v_cmd - dv, self.v_cmd + dv)

    def next_step(self):
        self.idx += 1
        if self.idx >= len(self.steps):
            self.send(0.0, 0.0)
            if not self.done:
                self.done = True
                self.pub_done.publish(Bool(data=True))
                rospy.loginfo("Kịch bản hoàn tất.")
            return
        s = self.steps[self.idx]
        self.st = {"t0": rospy.get_time(), "start": self.pose, "phase": 0, "acc_yaw": 0.0, "last_yaw": self.pose[2]}
        if "goto" in s:
            self.st["from"] = self.prev_xy or self.pose[:2]
        label = s.get("label") or ", ".join("%s=%s" % kv for kv in s.items())
        self.pub_step.publish(String(data=label))
        rospy.loginfo("[%d/%d] %s", self.idx + 1, len(self.steps), label)

    def tick(self, _):
        if self.pose is None or self.done:
            return
        if self.t_start is None:
            self.t_start = rospy.get_time()
        if self.idx < 0:
            if rospy.get_time() - self.t_start < self.sc.get("start_delay", 5.0) or not rospy.get_param("~autostart", True):
                self.send(0.0, 0.0)
                return
            self.next_step()
            if self.done:
                return
        s = self.steps[self.idx]
        x, y, th = self.pose
        if "goto" in s:
            fin = self.do_goto(s, x, y, th)
        elif "spin" in s:
            fin = self.do_spin(s, th)
        elif "face" in s:
            fin = self.do_face(s["face"], th)
        elif "dash" in s:
            fin = self.do_dash(s["dash"], x, y)
        elif "wait" in s:
            self.send(0.0, 0.0)
            fin = rospy.get_time() - self.st["t0"] >= s["wait"]
        else:
            rospy.logwarn("Bước không hợp lệ: %s", s)
            fin = True
        if fin:
            self.next_step()

    def do_face(self, yaw, th):
        err = wrap(yaw - th)
        if abs(err) < 0.02 and abs(self.w_cmd) < 0.3:
            self.send(0.0, 0.0)
            return True
        self.send(self.ramp(0.0), clamp(1.5 * err, -0.8, 0.8))
        return False

    def do_goto(self, s, x, y, th):
        gx, gy = s["goto"]
        fx, fy = self.st["from"]
        vmax = s.get("v", self.v_def)
        dist = math.hypot(gx - x, gy - y)
        lh = math.atan2(gy - fy, gx - fx) if math.hypot(gx - fx, gy - fy) > 0.05 else math.atan2(gy - y, gx - x)
        if self.st["phase"] == 0:
            if dist < 0.05 or self.do_face(math.atan2(gy - y, gx - x), th):
                self.st["phase"] = 1
            return False
        ex, ey = x - fx, y - fy
        cross = -math.sin(lh) * ex + math.cos(lh) * ey
        along_left = (gx - x) * math.cos(lh) + (gy - y) * math.sin(lh)
        v = self.ramp(clamp(min(vmax, 0.8 * along_left), 0.0, vmax))
        hd = wrap(lh - math.atan2(1.5 * cross, max(v, 0.2)) - th)
        self.send(v, clamp(2.0 * hd, -0.8, 0.8))
        if along_left < 0.03:
            self.send(0.0, 0.0)
            self.prev_xy = (gx, gy)
            return True
        return False

    def do_spin(self, s, th):
        turns = s["spin"]
        w = abs(s.get("w", 1.0)) * (1 if turns >= 0 else -1)
        self.st["acc_yaw"] += wrap(th - self.st["last_yaw"])
        self.st["last_yaw"] = th
        remain = 2 * math.pi * abs(turns) - abs(self.st["acc_yaw"])
        if remain <= 0.02:
            self.send(0.0, 0.0)
            self.prev_xy = self.pose[:2]
            return True
        self.send(0.0, w if remain > 0.4 else w * max(remain / 0.4, 0.15))
        return False

    def do_dash(self, d, x, y):
        sx, sy, _ = self.st["start"]
        if self.st["phase"] == 0:
            if math.hypot(x - sx, y - sy) >= d["dist"]:
                self.st["phase"] = 1
                self.v_cmd = 0.0
                self.send(0.0, 0.0)  # PHANH GẤP
                rospy.loginfo("  -> phanh gấp")
                return False
            self.send(min(d["v"], self.v_cmd + 1.5 * self.DT), 0.0)
            return False
        self.send(0.0, 0.0)
        self.prev_xy = self.pose[:2]
        return True


if __name__ == "__main__":
    rospy.init_node("scenario_runner")
    n = ScenarioRunner()
    rospy.on_shutdown(lambda: n.send(0.0, 0.0))
    rospy.spin()
