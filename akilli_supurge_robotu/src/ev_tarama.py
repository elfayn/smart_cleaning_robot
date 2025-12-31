#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
import time
import random

# =====================
# HIZ (YÜKSEK AMA GÜVENLİ)
# =====================
LIN = 0.45        # ileri hız (çok hızlı)
ANG = 1.6         # dönüş hızı (atak)

FRONT_LIMIT = 0.65
SIDE_LIMIT  = 0.75

front = left = right = 10.0
last_progress_time = time.time()

# =====================
# LIDAR CALLBACK
# =====================
def scan_cb(msg):
    global front, left, right
    ranges = msg.ranges

    front_ranges = ranges[0:20] + ranges[-20:]
    left_ranges  = ranges[60:100]
    right_ranges = ranges[260:300]

    def clean(arr):
        return min([r for r in arr if not math.isinf(r)], default=10.0)

    front = clean(front_ranges)
    left  = clean(left_ranges)
    right = clean(right_ranges)

# =====================
# ROS INIT
# =====================
rospy.init_node("ev_kesif")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/scan", LaserScan, scan_cb)

rate = rospy.Rate(12)   # daha hızlı loop
twist = Twist()

rospy.sleep(2.0)

# =====================
# MAIN LOOP
# =====================
while not rospy.is_shutdown():
    now = time.time()
    moving_forward = False

    # 🚪 KAPI / AÇIK ALAN → TAM GAZ
    if front > FRONT_LIMIT and right > SIDE_LIMIT:
        twist.linear.x = LIN
        twist.angular.z = 0.0
        moving_forward = True

    # 🧱 ÖN KAPALI → SERT SOLA DÖN
    elif front < FRONT_LIMIT:
        twist.linear.x = 0.08
        twist.angular.z = ANG

    # 🧭 DUVAR TAKİBİ (HIZLI)
    else:
        twist.linear.x = LIN * 0.9
        twist.angular.z = -ANG * 0.45
        moving_forward = True

    # ✅ İLERLEME VARSA ZAMAN SIFIRLA
    if moving_forward:
        last_progress_time = now

    # 🚨 SIKIŞTI → AGRESİF KURTARMA
    if now - last_progress_time > 3.0:
        twist.linear.x = -0.05
        twist.angular.z = random.choice([-1, 1]) * ANG
        last_progress_time = now

    pub.publish(twist)
    rate.sleep()

