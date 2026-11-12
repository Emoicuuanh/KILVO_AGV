#!/usr/bin/env python3
"""Ghi quỹ đạo ra file TUM + log vùng/trượt (ROS 1).

~output_dir       mặc định /ws/results/<thời gian>
~estimate_topics  danh sách topic nav_msgs/Odometry, mặc định ['/aft_mapped_to_init', '/wheel/odom']
~stop_on_done     tự tắt khi /sim/scenario_done = True
"""
import os
import time

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, String


class TrajRecorder:
    # ===== KHỞI TẠO: tạo thư mục kết quả, mở file, đăng ký các topic cần ghi =====
    def __init__(self):
        out = rospy.get_param("~output_dir", "") or os.path.join("/ws/results", time.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(out, exist_ok=True)
        self.out, self.files = out, []
        # --- Quỹ đạo thật ---
        f_gt = self._open("ground_truth.tum")
        rospy.Subscriber("/ground_truth/odom", Odometry, lambda m: self.w(f_gt, m), queue_size=500)
        # --- Các quỹ đạo ước lượng: mỗi topic Odometry -> 1 file .tum (tên = tên topic) ---
        for tp in rospy.get_param("~estimate_topics", ["/aft_mapped_to_init", "/wheel/odom"]):
            f = self._open(tp.strip("/").replace("/", "_") + ".tum")
            rospy.Subscriber(tp, Odometry, lambda m, f=f: self.w(f, m), queue_size=500)
        # --- Nhật ký sự kiện: vùng hiện tại, bước kịch bản, trạng thái trượt ---
        self.f_ev = self._open("events.csv")
        self.f_ev.write("t,kind,value\n")
        rospy.Subscriber("/sim/zone", String, lambda m: self.ev("zone", m.data), queue_size=20)
        rospy.Subscriber("/sim/step", String, lambda m: self.ev("step", m.data.replace(",", ";")), queue_size=20)
        rospy.Subscriber("/sim/slip", Float32MultiArray,
                         lambda m: self.ev("slip", " ".join("%.4f" % v for v in m.data)), queue_size=100)
        # --- Tự dừng khi kịch bản xong; đóng file khi tắt node ---
        rospy.Subscriber("/sim/scenario_done", Bool, self.cb_done, queue_size=1)
        rospy.on_shutdown(self.close)
        rospy.loginfo("traj_recorder: kết quả lưu tại %s", out)

    # ===== GHI FILE =====
    def _open(self, name):
        f = open(os.path.join(self.out, name), "w", buffering=1)
        self.files.append(f)
        return f

    # 1 dòng TUM: thời_gian x y z qx qy qz qw
    @staticmethod
    def w(f, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if not f.closed:
            f.write("%.6f %.6f %.6f %.6f %.7f %.7f %.7f %.7f\n" % (m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def ev(self, kind, val):
        if not self.f_ev.closed:
            self.f_ev.write("%.4f,%s,%s\n" % (rospy.get_time(), kind, val))

    # ===== KẾT THÚC =====
    def cb_done(self, m):
        if m.data and rospy.get_param("~stop_on_done", True):
            rospy.loginfo("Kịch bản xong. Đánh giá: rosrun kilvo_agv_gazebo eval_ate.py %s", self.out)
            rospy.signal_shutdown("scenario done")

    def close(self):
        for f in self.files:
            if not f.closed:
                f.close()


# ===== MAIN =====
if __name__ == "__main__":
    rospy.init_node("traj_recorder")
    TrajRecorder()
    rospy.spin()
