#!/usr/bin/env python3
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#Modified by my_name_is_D
import math
import numpy
import rospy
import tf.transformations as trans
from PIDRegulator import PIDRegulator, QuaternionPIDRegulator

from dynamic_reconfigure.server import Server
from osmose_description.cfg import PositionControlConfig
from osmose_description.msg import SetTraj
import geometry_msgs.msg as geometry_msgs
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
from scipy.spatial.transform import Rotation


class PositionControllerNode:
    def __init__(self):
        print('PositionControllerNode: initializing node')

        self.config = {}
        self.velocity_gain = float(rospy.get_param("position_control/velocity_gain"))

        self.pos_des = numpy.array([])
        self.quat_des = numpy.array([0, 0, 0, 1])
        self.position = numpy.array([])
        self.orientation = numpy.array([0, 0, 0, 1])
        self.ang_v= numpy.zeros(3)
        self.t = rospy.get_time()

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDRegulator(1, 0, 0, 1)
        self.pid_pos = QuaternionPIDRegulator(1, 0, 0, 1)
        

        # ROS infrastructure
        #sub_cmd_pose = rospy.Subscriber('cmd_pose', numpy_msg(geometry_msgs.PoseStamped), self.cmd_pose_callback)
        sub_cmd_traj = rospy.Subscriber('set_traj', SetTraj, self.cmd_traj_callback)
        sub_odometry = rospy.Subscriber('odom', numpy_msg(Odometry), self.odometry_callback)
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', geometry_msgs.Twist, queue_size=10)
        self.srv_reconfigure = Server(PositionControlConfig, self.config_callback)

        #execution Timer, so ze can decide the delay between 2 process
        timer_excution_delay = 1000000 #ns
        self.info_process_timer = rospy.Timer(rospy.Duration(nsecs=timer_excution_delay), self.execution)


    def cmd_pose_callback(self, msg):
        """Handle updated set pose callback."""
        # Just store the desired pose. The actual control runs on odometry callbacks
        p = msg.pose.position 
        q = msg.pose.orientation 
        self.pos_des = numpy.array([p.x, p.y,self.position[2]])
        self.quat_des = numpy.array([q.x, q.y, q.z, q.w])

    def cmd_traj_callback(self,msg):
        '''Handle updated traj cmd'''
        '''0.eTrajType_Unknown|
1.eTrajType_Goto| (x,y,direction)
2.eTrajType_Distance| (distance mm)
3.eTrajType_Rotate| (angle deg)
4.eTrajType_RotateReal| (angle deg)
5.eTrajType_RotateToXY| (x, y, direction)
6.eTrajType_Border| (angleEnslavement (bool), maxspeed, direction)
7.eTrajType_Stop (soft_hard (int))
'''
        if msg.trajtype == 0:
            pass
        elif msg.trajtype ==1:
            #Position
            self.pos_des = numpy.array([msg.position[0]* 0.01, msg.position[1]* 0.01,self.position[2]* 0.01])
            #direction 
            direction =1 #?????????

        elif msg.trajtype ==2:
            self.pos_des[0] = self.position[0] + msg.distance* 0.01
            self.pos_des[1] = self.position[1] 
            self.pos_des[2]= self.position[2]

        elif msg.trajtype ==3 or msg.trajtype ==4: #????????????????????
            rot = Rotation.from_euler('xyz', [0, 0, msg.angle], degrees=True)
            self.quat_des = rot.as_quat()

        elif msg.trajtype == 5:
            angle_desired = numpy.arctan2((msg.position[1]* 0.01-self.position[1]) / (msg.position[0]* 0.01-self.position[0]) )
            rot = Rotation.from_euler('xyz', [0, 0, angle_desired], degrees=True)
            self.quat_des = rot.as_quat()

        elif msg.trajtype == 6: #????????????????????
            angle_enslavment = msg.angle_enslavment
            max_speed = msg.max_speed_or_soft_hard
            direction = msg.direction

        elif msg.trajtype == 7: #????????????????????
            soft_hard = msg.max_speed_or_soft_hard
            





    def odometry_callback(self, msg):
        """Handle updated measured velocity callback."""
        if not bool(self.config):
            return
        
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.position = numpy.array([p.x, p.y,p.z])
        self.orientation = numpy.array([q.x, q.y, q.z, q.w])
        self.ang_v= numpy.array([msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z])
        self.t = msg.header.stamp.to_sec()

    def config_callback(self, config, level):
        """Handle updated configuration values."""
        # Config has changed, reset PID controllers
        self.pid_pos = PIDRegulator(config['pos_p'], config['pos_i'], config['pos_d'], config['pos_sat'])
        self.pid_rot = QuaternionPIDRegulator(config['rot_p'], config['rot_i'], config['rot_d'], config['rot_sat'])
        self.config = config

        return config

    def check_param(self):
        """ check all the variable speed and stuff"""
        pass

    def execution(self, event):
        """ Check if msg was updated and send configurations in simulation"""

        if not self.initialized:
            # If this is the first callback: Store and hold latest pose.
            self.pos_des  = self.position
            self.quat_des = self.orientation
            self.initialized = True

        self.check_param()
        
        if len(self.position) > 1 and len(self.pos_des) > 1 :
            # Position error
            e_pos_world = self.pos_des - self.position
            print("des et pos",self.pos_des,self.position)
            e_pos_body = trans.quaternion_matrix(self.orientation).transpose()[0:3,0:3].dot(e_pos_world)

            # Error quaternion wrt body frame
            # e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), self.quat_des)
            #WE don't care in an arena of 2m width 
            # if numpy.linalg.norm(e_pos_world[0:2]) > 5.0:
            #     # special case if we are far away from goal:
            #     # ignore desired heading, look towards goal position
            #     heading = math.atan2(e_pos_world[1],e_pos_world[0])
            #     quat_des = numpy.array([0, 0, math.sin(0.5*heading), math.cos(0.5*heading)])
            #     e_rot_quat = trans.quaternion_multiply(trans.quaternion_conjugate(q), quat_des)
            # Error angles
            # e_rot = numpy.array(trans.euler_from_quaternion(e_rot_quat))


            v_linear = self.pid_pos.regulate(e_pos_body, self.t)  * self.velocity_gain  #We can reduce the output of the cmd_vel if desired
            v_angular = self.pid_rot.regulate(self.quat_des,self.orientation, self.ang_v,self.t)

            # Convert and publish vel. command:
            cmd_vel = geometry_msgs.Twist()
            cmd_vel.linear = geometry_msgs.Vector3(*v_linear)
            cmd_vel.angular = geometry_msgs.Vector3(*v_angular)

            self.pub_cmd_vel.publish(cmd_vel)

            if numpy.linalg.norm(e_pos_world[0:1]) < 0.05: #5cm
                print("goal reached") #TO DO: send a real msg with end
            




if __name__ == '__main__':
    print('starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')


