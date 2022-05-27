#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np
import math

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):

        #Initialisations
        l1  = self.l1
        l2  = self.l2
        l3  = self.l3
        l4  = self.l4
        l5  = self.l5

        th1 = self.theta1
        th2 = self.theta2

        x = l4 * math.sin(th1)
        y = l4 * math.cos(th1)
        z = l5 * math.sin(th2)
        w = l5 * math.cos(th2)

        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        c1 = math.cos(q1)
        c2 = math.cos(q2)
        c3 = math.cos(q3)
        c4 = math.cos(q4)
        c5 = math.cos(q5)
        c6 = math.cos(q6)
        c7 = math.cos(q7)

        s1 = math.sin(q1)
        s2 = math.sin(q2)
        s3 = math.sin(q3)
        s4 = math.sin(q4)
        s5 = math.sin(q5)
        s6 = math.sin(q6)
        s7 = math.sin(q7)

        ########################

        J_11 = -l3*(c1*s3 + c2*c3*s1) - x*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - y*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - w*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - z*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - l2*s1*s2

        J_12 = x*(c1*c2*s4 - c1*c3*c4*s2) - z*(s6*(c1*c2*c4 + c1*c3*s2*s4) - c6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5)) - w*(c6*(c1*c2*c4 + c1*c3*s2*s4) + s6*(c5*(c1*c2*s4 - c1*c3*c4*s2) - c1*s2*s3*s5)) - y*(c1*c2*c4 + c1*c3*s2*s4) + l2*c1*c2 - l3*c1*c3*s2

        J_13 = w*(s6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) - c6*s4*(c3*s1 + c1*c2*s3)) - l3*(c3*s1 + c1*c2*s3) - z*(c6*(s5*(s1*s3 - c1*c2*c3) + c4*c5*(c3*s1 + c1*c2*s3)) + s4*s6*(c3*s1 + c1*c2*s3)) - x*c4*(c3*s1 + c1*c2*s3) - y*s4*(c3*s1 + c1*c2*s3)

        J_14 = x*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - z*(s6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - c5*c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - w*(c6*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) + c5*s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2)) - y*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4)

        J_15 = (z*c6 - w*s6)*(c3*c5*s1 + c1*c2*c5*s3 - c1*s2*s4*s5 + c4*s1*s3*s5 - c1*c2*c3*c4*s5)

        J_16 = w*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - z*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3)))

        J_17 = 0

        ########################

        J_21 = l2*c1*s2 - x*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - y*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - w*(c6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) - s6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - z*(s6*(s4*(s1*s3 - c1*c2*c3) + c1*c4*s2) + c6*(c5*(c4*(s1*s3 - c1*c2*c3) - c1*s2*s4) - s5*(c3*s1 + c1*c2*s3))) - l3*(s1*s3 - c1*c2*c3)

        J_22 = x*(c2*s1*s4 - c3*c4*s1*s2) - y*(c2*c4*s1 + c3*s1*s2*s4) - w*(c6*(c2*c4*s1 + c3*s1*s2*s4) + s6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5)) - z*(s6*(c2*c4*s1 + c3*s1*s2*s4) - c6*(c5*(c2*s1*s4 - c3*c4*s1*s2) - s1*s2*s3*s5)) + l2*c2*s1 - l3*c3*s1*s2

        J_23 = l3*(c1*c3 - c2*s1*s3) - w*(s6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) - c6*s4*(c1*c3 - c2*s1*s3)) + z*(c6*(s5*(c1*s3 + c2*c3*s1) + c4*c5*(c1*c3 - c2*s1*s3)) + s4*s6*(c1*c3 - c2*s1*s3)) + x*c4*(c1*c3 - c2*s1*s3) + y*s4*(c1*c3 - c2*s1*s3)

        J_24 = w*(c6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) + c5*s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) + z*(s6*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - c5*c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2)) - x*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + y*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4)

        J_25 = -(z*c6 - w*s6)*(c1*c3*c5 - c2*c5*s1*s3 + c1*c4*s3*s5 + s1*s2*s4*s5 + c2*c3*c4*s1*s5)

        J_26 = z*(c6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) - s6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3))) - w*(s6*(s4*(c1*s3 + c2*c3*s1) - c4*s1*s2) + c6*(c5*(c4*(c1*s3 + c2*c3*s1) + s1*s2*s4) - s5*(c1*c3 - c2*s1*s3)))

        J_27 = 0

        ########################

        J_31 = 0

        J_32 = w*(s6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) + c6*(c4*s2 - c2*c3*s4)) - z*(c6*(c5*(s2*s4 + c2*c3*c4) + c2*s3*s5) - s6*(c4*s2 - c2*c3*s4)) - x*(s2*s4 + c2*c3*c4) + y*(c4*s2 - c2*c3*s4) - l2*s2 - l3*c2*c3

        J_33 = s2*(l3*s3 + x*c4*s3 + y*s3*s4 - z*c3*c6*s5 + w*c6*s3*s4 + w*c3*s5*s6 + z*s3*s4*s6 + z*c4*c5*c6*s3 - w*c4*c5*s3*s6)

        J_34 = x*(c2*c4 + c3*s2*s4) + y*(c2*s4 - c3*c4*s2) + w*(c6*(c2*s4 - c3*c4*s2) - c5*s6*(c2*c4 + c3*s2*s4)) + z*(s6*(c2*s4 - c3*c4*s2) + c5*c6*(c2*c4 + c3*s2*s4))

        J_35 = -(z*c6 - w*s6)*(c5*s2*s3 + c2*s4*s5 - c3*c4*s2*s5)

        J_36 = - w*(c6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) - s6*(c2*c4 + c3*s2*s4)) - z*(s6*(c5*(c2*s4 - c3*c4*s2) - s2*s3*s5) + c6*(c2*c4 + c3*s2*s4))
        
        J_37 = 0

        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J

    def tf_A01(self, r_joints_array):

        q1 = r_joints_array[0]
        l1 = self.l1

        c1 = math.cos(q1)
        s1 = math.sin(q1)

        tf = np.matrix([[c1 , -s1 , 0 , 0],\
                        [s1 ,  c1 , 0 , 0],\
                        [0  ,  0  , 1 , l1],\
                        [0  ,  0  , 0 , 1]])
        return tf

    def tf_A02(self, r_joints_array):


        q2 = r_joints_array[1]
        c2 = math.cos(q2)
        s2 = math.sin(q2)

        tf_A12= np.matrix([[c2 , -s2 , 0 , 0],\
                          [0   ,  0  , 1 , 0],\
                          [-s2 , -c2 , 0 , 0],\
                          [0   ,  0  , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):

        q3 = r_joints_array[2]
        l2 = self.l2

        c3 = math.cos(q3)
        s3 = math.sin(q3)


        tf_A23 = np.matrix([[c3 , -s3 ,  0 ,  0],\
                           [0   ,  0  , -1 , -l2],\
                           [s3  ,  c3 ,  0 ,  0],\
                           [0   ,  0  ,  0 ,  1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        
        q4 = r_joints_array[3]
        l3 = self.l3

        c4 = math.cos(q4)
        s4 = math.sin(q4)

        tf_A34 = np.matrix([[c4 , -s4 ,  0 ,  l3],\
                           [  0 ,  0  , -1 ,  0],\
                           [ s4 ,  c4 ,  0 ,  0],\
                           [0   ,  0  ,  0 ,  1]])

        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):

        l4  = self.l4
        th1 = self.theta1

        x = l4 * math.sin(th1)
        y = l4 * math.cos(th1)

        q5 = r_joints_array[4]

        c5 = math.cos(q5)
        s5 = math.sin(q5)
        
        tf_A45 = np.matrix([[c5 , -s5 , 0 ,  x],\
                           [  0 ,  0  ,-1 , -y],\
                           [ s5 ,  c5 , 0 ,  0],\
                           [0   ,  0  , 0 ,  1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        
        q6 = r_joints_array[5]

        c6 = math.cos(q6)
        s6 = math.sin(q6)
        
        tf_A56 = np.matrix([[c6 , -s6 ,  0 ,  0],\
                           [0   ,  0  , -1 ,  0],\
                           [s6  ,  c6 ,  0 ,  0],\
                           [0   ,  0  ,  0 ,  1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):

        l5  = self.l5
        th2 = self.theta2

        z = l5 * math.sin(th2)
        w = l5 * math.cos(th2)

        q7 = r_joints_array[6]

        c7 = math.cos(q7)
        s7 = math.sin(q7)

        tf_A67 = np.matrix([[c7 , -s7 , 0 ,  z],\
                           [  0 ,  0  , 1 ,  w],\
                           [-s7 , -c7 , 0 ,  0],\
                           [0   ,  0  , 0 ,  1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
