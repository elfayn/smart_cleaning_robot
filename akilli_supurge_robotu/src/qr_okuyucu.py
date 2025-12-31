#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class FakeQROkuyucu:
    def __init__(self):
        rospy.init_node("qr_okuyucu")

        self.pub = rospy.Publisher("/qr_result", String, queue_size=10)

        rospy.loginfo("QR Okuyucu baslatildi")

        rospy.Timer(rospy.Duration(2.0), self.publish_fake_qr)

    def publish_fake_qr(self, event):
        # sırayla oda isimleri gönderelim (demo için yeterli)
        odalar = [
            "ROOM=LIVINGROOM",
            "ROOM=KITCHEN",
            "ROOM=BEDROOM",
            "ROOM=CORRIDOR"
        ]
        msg = String()
        msg.data = odalar[int(rospy.get_time()) % 4]
        self.pub.publish(msg)
        rospy.loginfo(f"QR okundu (FAKE): {msg.data}")

if __name__ == "__main__":
    FakeQROkuyucu()
    rospy.spin()

