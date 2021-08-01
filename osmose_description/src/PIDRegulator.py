# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
# Refactored by uWare Robotics (my_name_is_D)

# Based upon:
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

import numpy

#Remastered for osmose (actually reverted to uuv_simulator design)
class PIDRegulator:
    """A very basic 1D PID Regulator."""
    def __init__(self, p, i, d, sat):
        self.p = p
        self.i = i
        self.d = d
        self.sat = sat

        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def __str__(self):
        msg = 'PID controller:'
        msg += '\n\tp=%f' % self.p
        msg += '\n\ti=%f' % self.i
        msg += '\n\td=%f' % self.d
        msg += '\n\tsat=%f' % self.sat
        return msg

    def regulate(self, err, t):
        derr_dt = 0.0
        dt = t - self.prev_t
        if self.prev_t > 0.0 and dt > 0.0:
            derr_dt = (err - self.prev_err)/dt
            self.integral += 0.5*(err + self.prev_err)*dt

        u = self.p*err + self.d*derr_dt + self.i*self.integral

        self.prev_err = err
        self.prev_t = t

        if (numpy.linalg.norm(u) > self.sat):
            # controller is in saturation: limit outpt, reset integral
            u = self.sat*u/numpy.linalg.norm(u)
            self.integral = 0.0

        return u


class OpenLoopRegulator :
    """Open loop controller implemented for the linear velocity controller"""
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c

    def __str__(self):
        msg = 'Openloop controller reset'
        msg += '\n\ta=%s' % str(self.a)
        msg += '\n\tb=%s' % str(self.b)
        msg += '\n\tc=%s' % str(self.c)
        return msg

    def reset(self):
        return

    def saturate(self, saturating):
        return

    def regulate(self, ref, t):
        u_lin = numpy.sign(ref) * pow(ref, 2) * self.a + \
                ref * self.b + \
                self.c 
        return u_lin

#SIMPLIFIED VERSION FOR OSMOSE
class QuaternionPIDRegulator :
    """Quaternion controller implemented according to NASA paper"""
    def __init__(self, p, i, d, sat):
        self.p = numpy.array([[p, 0, 0, 0],[0, p, 0, 0], [0, 0, p, 0], [0, 0, 0, p]])
        self.i = numpy.array([[i, 0, 0, 0],[0, i, 0, 0], [0, 0, i, 0], [0, 0, 0, i]])
        self.d = numpy.array([[d, 0, 0, 0],[0, d, 0, 0], [0, 0, d, 0], [0, 0, 0, d]])
        # TODO: Saturation function
        self.sat = sat
        self.reset()
    
    def reset(self):
        self.integral = 0
        self.prev_err = 0
        self.prev_t = -1.0

    def regulate(self, q_ref, q_mes, w_mes, t):
        
        q_ref = [q_ref[3],q_ref[0],q_ref[1],q_ref[2]]
        q_mes = [q_mes[3],q_mes[0],q_mes[1],q_mes[2]]
        
        q_err = self.quaternion_product(self.invert_quaternion(q_mes), q_ref)
        if q_err[0] < 0 :
            q_err = -q_err
        q_err_track = numpy.array([1, 0, 0, 0]) - q_err
        
        # Mathematical trick for quaternion product (see Nasa paper)
        Q_left_ref = self.quaternion_left_product(q_ref)
        Q_left_error = self.quaternion_left_product(q_err)
        Q_right_error = self.quaternion_right_product(q_err)

        # Get angular reference velocity
        w_ref = 2*numpy.transpose(Q_left_ref).dot(numpy.zeros(4))

        # Get error derivative
        q_err_track_dot = - 0.5*(- Q_right_error.dot(w_mes))

        # Command law
        q_ref_dot = Q_left_error.dot(numpy.transpose(Q_left_error)).dot(self.p.dot(q_err_track) + self.d.dot(q_err_track_dot))
        u = -numpy.transpose(Q_right_error).dot((2*q_ref_dot))
        
        return u

    def quaternion_product(self, p, q):
        result = numpy.array([p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3], 
                        p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2], 
                        p[0]*q[2] + p[2]*q[0] + p[3]*q[1] - p[1]*q[3], 
                        p[0]*q[3] + p[3]*q[0] + p[1]*q[2] - p[2]*q[1]])
        return result

    def invert_quaternion(self, q):
        result = numpy.array([q[0], -q[1], -q[2], -q[3]])
        return result

    def quaternion_left_product(self, q):
        """
        Returns an equivalent matrix Q_left so that : 
        quaternion_product([q0, q1, q2, q3], [0, wx, wy, wz]) = Q_left.dot([wx, wy, wz])
        """
        Q_left = numpy.array([[-q[1], -q[2], -q[3]], 
                         [q[0]  , -q[3],  q[2]], 
                         [q[3] ,  q[0] , -q[1]], 
                         [-q[2],  q[1],  q[0] ]])
        return Q_left

    def quaternion_right_product(self, q):
        """
        Returns an equivalent matrix Q_right so that : 
        quaternion_product([0, wx, wy, wz], [q0, q1, q2, q3]) = Q_right.dot([wx, wy, wz])
        """
        Q_right = numpy.array([ [-q[1], -q[2], -q[3]], 
                            [q[0], q[3], -q[2]], 
                            [-q[3], q[0], q[1]], 
                            [q[2], -q[1], q[0]]])
        return Q_right