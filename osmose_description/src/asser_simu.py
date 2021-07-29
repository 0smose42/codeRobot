#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class asser_simu():
    def __init__():
    
    sub_odom=rospy.Subscriber("odom",Odometry,self.odom_callback)
    self.position = [0,0,0]
    self.orientation  = [0,0,0,1]

    def odom_callback(self, msg):
        self.position = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
        self.orientation  = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z, msg.pose.orientation.w]

    def orientation_to_ang_vel(self):



if __name__ == '__main__':
    rospy.init_node('asser_simu', anonymous=True)
    print('Starting asser_simu')

    try:
        node = asser_simu()
        rospy.spin()

    except rospy.ROSInterruptException:
        print('asser_simu::Exception')
    print('Closing asser_simu')