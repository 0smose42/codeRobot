#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class mockup():
    def __init__():
    self.pub = rospy.Publisher('chatter', String, queue_size=10)


    def talker(self):
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        self.pub.publish(hello_str)
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    node = mockup()
    while not rospy.is_shutdown():
        node.my_code()
        rate.sleep()

#---------------------------

#2 ways to have a node running
# while not rospy.is_shutdown():  ### While no one killed the node continue to iterate in here
#OR
#rospy.spin()   ### While no one killed the node continue to iterate in here AT THE CLOCK of the data you receive
#(so if you subscribe, usefull not to run the node real fast endlessly)
