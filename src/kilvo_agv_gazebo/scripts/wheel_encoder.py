#!/usr/bin/env python3
"""Encoder bánh xe + odometry bánh + nhãn trượt ground-truth (ROS 1).

Vào : /joint_states (góc bánh từ diff_drive plugin), /ground_truth/odom
Ra  : /wheel/encoder  JointState (góc đã lượng tử theo xung, vận tốc góc bánh)
      /wheel/twist    TwistWithCovarianceStamped (v, w thân xe từ encoder, frame base_link)
      /wheel/odom     Odometry dead-reckoning chỉ từ encoder (baseline)
      /sim/slip       Float32MultiArray [v_enc, w_enc, v_gt, w_gt, slip_L, slip_R, flag]
      TF world->base_link (tư thế thật, ~publish_gt_tf) để xem RViz
"""
import bisect
import math

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
from agv_common import wrap, yaw_from_quat


class WheelEncoder:
    # ===== KHỞI TẠO =====
    def __init__(self):
        # --- Tham số (đặt qua _ten:=giatri hoặc <param> trong launch) ---
        #   ticks_per_rev: số xung/vòng của encoder; rate: tần số xuất (Hz)
        #   wheel_radius/separation: giá trị THẬT của robot mô phỏng
        #   radius_scale_*/separation_scale: hệ số sai lệch "danh định" để giả lập hiệu chuẩn sai (vd. bánh mòn)
        g = lambda n, d: rospy.get_param("~" + n, d)  # noqa: E731
        self.lj, self.rj = g("left_joint", "left_wheel_joint"), g("right_joint", "right_wheel_joint")
        self.cpr = int(g("ticks_per_rev", 4096))
        self.period = 1.0 / g("rate", 50.0)
        self.r_true, self.b_true = g("wheel_radius", 0.10), g("wheel_separation", 0.50)
        self.rl = self.r_true * g("radius_scale_left", 1.0)
        self.rr = self.r_true * g("radius_scale_right", 1.0)
        self.b = self.b_true * g("separation_scale", 1.0)
        self.slip_thr = g("slip_threshold", 0.05)
        self.tfb = tf2_ros.TransformBroadcaster() if g("publish_gt_tf", True) else None
        # --- Publisher: các topic đầu ra ---
        self.pub_enc = rospy.Publisher("/wheel/encoder", JointState, queue_size=50)
        self.pub_tw = rospy.Publisher("/wheel/twist", TwistWithCovarianceStamped, queue_size=50)
        self.pub_odom = rospy.Publisher("/wheel/odom", Odometry, queue_size=50)
        self.pub_slip = rospy.Publisher("/sim/slip", Float32MultiArray, queue_size=50)
        # --- Trạng thái ---
        self.gt_t, self.gt_pose = [], []   # bộ đệm ground truth (thời gian, (x, y, yaw)) để nội suy
        self.last = None                   # mẫu encoder trước: (t, xung trái, xung phải, pose GT)
        self.x = self.y = self.th = 0.0    # tư thế odometry bánh (dead-reckoning)
        self.odom_init = False
        self.last_tf = -1.0
        # --- Subscriber: đầu vào ---
        rospy.Subscriber("/ground_truth/odom", Odometry, self.cb_gt, queue_size=200)
        rospy.Subscriber("/joint_states", JointState, self.cb_joint, queue_size=100)

    # ===== NHẬN GROUND TRUTH =====
    # Lưu vào bộ đệm; lần đầu dùng làm điểm xuất phát của odometry bánh; xuất TF world->base_link.
    def cb_gt(self, m):
        t = m.header.stamp.to_sec()
        pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw_from_quat(m.pose.pose.orientation))
        if self.gt_t and t <= self.gt_t[-1]:
            if t < self.gt_t[-1] - 1.0:
                self.gt_t, self.gt_pose = [], []
            else:
                return
        self.gt_t.append(t)
        self.gt_pose.append(pose)
        if len(self.gt_t) > 2000:
            del self.gt_t[:1000], self.gt_pose[:1000]
        if not self.odom_init:
            self.x, self.y, self.th = pose
            self.odom_init = True
        if self.tfb and t - self.last_tf >= 0.02:
            self.last_tf = t
            tf = TransformStamped()
            tf.header.stamp = m.header.stamp
            tf.header.frame_id = "world"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = m.pose.pose.position.x
            tf.transform.translation.y = m.pose.pose.position.y
            tf.transform.translation.z = m.pose.pose.position.z
            tf.transform.rotation = m.pose.pose.orientation
            self.tfb.sendTransform(tf)

    # ===== NỘI SUY GROUND TRUTH tại thời điểm t (GT 100 Hz, encoder 50 Hz không trùng nhịp) =====
    def gt_at(self, t):
        if not self.gt_t or t < self.gt_t[0] or t > self.gt_t[-1] + 0.05:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i >= len(self.gt_t):
            return self.gt_pose[-1]
        if i == 0:
            return self.gt_pose[0]
        t0, t1 = self.gt_t[i - 1], self.gt_t[i]
        a = (t - t0) / max(t1 - t0, 1e-9)
        p0, p1 = self.gt_pose[i - 1], self.gt_pose[i]
        return (p0[0] + a * (p1[0] - p0[0]), p0[1] + a * (p1[1] - p0[1]), wrap(p0[2] + a * wrap(p1[2] - p0[2])))

    # ===== NHẬN GÓC BÁNH -> ENCODER, ODOMETRY, NHÃN TRƯỢT =====
    def cb_joint(self, m):
        # --- B1. Đổi góc bánh (rad) thành số xung nguyên (giả lập độ phân giải encoder) ---
        if not self.odom_init or self.lj not in m.name or self.rj not in m.name:
            return
        il, ir = m.name.index(self.lj), m.name.index(self.rj)
        t = m.header.stamp.to_sec() or rospy.get_time()
        k = self.cpr / (2 * math.pi)
        tl, tr = math.floor(m.position[il] * k), math.floor(m.position[ir] * k)
        if self.last is None or t < self.last[0]:
            self.last = (t, tl, tr, self.gt_at(t))
            return
        dt = t - self.last[0]
        if dt < self.period - 1e-4:
            return
        # --- B2. Vận tốc góc bánh từ chênh lệch xung giữa 2 mẫu ---
        dql, dqr = (tl - self.last[1]) / k, (tr - self.last[2]) / k
        wl, wr = dql / dt, dqr / dt
        # --- B3. Động học vi sai: quãng đường 2 bánh -> tiến ds, quay dth -> cộng dồn tư thế ---
        dsl, dsr = dql * self.rl, dqr * self.rr
        ds, dth = 0.5 * (dsl + dsr), (dsr - dsl) / self.b
        self.x += ds * math.cos(self.th + 0.5 * dth)
        self.y += ds * math.sin(self.th + 0.5 * dth)
        self.th = wrap(self.th + dth)
        v_enc, w_enc = ds / dt, dth / dt

        # --- B4. Nhãn trượt thật: so tốc độ mặt bánh (r*omega) với tốc độ thật của điểm tiếp xúc ---
        #     slip > 0: bánh quay nhanh hơn xe thực đi (quay trơn); flag = 1 nếu vượt ngưỡng
        gt0, gt1 = self.last[3], self.gt_at(t)
        if gt0 is not None and gt1 is not None:
            dx, dy = gt1[0] - gt0[0], gt1[1] - gt0[1]
            c, s = math.cos(gt0[2]), math.sin(gt0[2])
            v_gt, w_gt = (c * dx + s * dy) / dt, wrap(gt1[2] - gt0[2]) / dt
            sl = wl * self.r_true - (v_gt - 0.5 * self.b_true * w_gt)
            sr = wr * self.r_true - (v_gt + 0.5 * self.b_true * w_gt)
            flag = 1.0 if max(abs(sl), abs(sr)) > self.slip_thr else 0.0
            self.pub_slip.publish(Float32MultiArray(data=[v_enc, w_enc, v_gt, w_gt, sl, sr, flag]))
        self.last = (t, tl, tr, gt1)

        # --- B5. Xuất /wheel/encoder (góc đã lượng tử + vận tốc góc bánh) ---
        js = JointState()
        js.header.stamp = m.header.stamp
        js.header.frame_id = "base_link"
        js.name = [self.lj, self.rj]
        js.position = [tl / k, tr / k]
        js.velocity = [wl, wr]
        self.pub_enc.publish(js)

        # --- B6. Xuất /wheel/twist: vận tốc thân xe + hiệp phương sai (vy, vz, wx, wy ~ 0 do không holonomic, phẳng) ---
        tw = TwistWithCovarianceStamped()
        tw.header = js.header
        tw.twist.twist.linear.x = v_enc
        tw.twist.twist.angular.z = w_enc
        q = (2 * math.pi / self.cpr / dt) * self.r_true
        cov = [0.0] * 36
        cov[0], cov[35] = (0.01 + q) ** 2, (0.02 + q / self.b) ** 2
        cov[7] = cov[14] = cov[21] = cov[28] = 1e-4
        tw.twist.covariance = cov
        self.pub_tw.publish(tw)

        # --- B7. Xuất /wheel/odom: tư thế dead-reckoning chỉ từ encoder (baseline để so sánh) ---
        od = Odometry()
        od.header.stamp = m.header.stamp
        od.header.frame_id = "world"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = self.x, self.y, 0.1
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(self.th / 2), math.cos(self.th / 2)
        od.twist.twist = tw.twist.twist
        self.pub_odom.publish(od)


# ===== MAIN =====
if __name__ == "__main__":
    rospy.init_node("wheel_encoder")
    WheelEncoder()
    rospy.spin()
