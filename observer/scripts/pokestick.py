#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import wiringpi2 as wiringpi

BUTTON = 26
INPUT = 0

wiringpi.wiringPiSetup()
wiringpi.pinMode(BUTTON, INPUT)

def talker():
    pub = rospy.Publisher('state', bool, queue_size=10)
    rospy.init_node('pokestick', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        button_state = wiringpi.digitalRead(BUTTON)
        pub.publish(button_state)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
