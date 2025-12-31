#!/usr/bin/env python3
import rospy
import yaml
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math

class GorevYoneticisi:
    def __init__(self):
        rospy.init_node("gorev_yoneticisi")

        self.qr_data = None

        rospy.Subscriber("/qr_result", String, self.qr_callback)

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("move_base bekleniyor...")
        self.client.wait_for_server()
        rospy.loginfo("move_base baglandi")

        self.load_mission()
        self.run_mission()

    def load_mission(self):
        path = rospy.get_param(
            "mission_file",
            "/home/elif/catkin_ws/src/akilli_supurge_robotu/config/gorev.yaml"
        )

        rospy.loginfo(f"Gorev dosyasi yukleniyor: {path}")
        with open(path, "r") as f:
            self.mission = yaml.safe_load(f)

    def qr_callback(self, msg):
        self.qr_data = msg.data
        rospy.loginfo(f"QR okundu: {self.qr_data}")

    def go_to(self, goal):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = "map"
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = goal["x"]
        g.target_pose.pose.position.y = goal["y"]

        yaw = goal.get("yaw", 0.0)
        g.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        g.target_pose.pose.orientation.z = math.sin(yaw / 2.0)

        self.client.send_goal(g)
        self.client.wait_for_result()

    def wait_for_qr(self, expected, timeout=10):
        start = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start < timeout:
            if self.qr_data == expected:
                return True
            rospy.sleep(0.5)
        return False

    def run_mission(self):
        rospy.loginfo("TEMIZLIK GOREVI BASLADI")

        for room in self.mission["rooms"]:
            rospy.loginfo(f"==== {room['name']} ====")

            rospy.loginfo("Oda girisine gidiliyor")
            self.go_to(room["entry_goal"])

            rospy.loginfo("QR bekleniyor...")
            if not self.wait_for_qr(room["qr_expected"]):
                rospy.logwarn("QR dogrulanamadi, oda atlandi")
                continue

            rospy.loginfo("QR dogrulandi, temizlik basliyor")

            for g in room["cleaning_goals"]:
                self.go_to(g)

            rospy.loginfo(f"{room['name']} temizlendi")

        rospy.loginfo("TEMIZLIK TAMAMLANDI")

if __name__ == "__main__":
    GorevYoneticisi()

