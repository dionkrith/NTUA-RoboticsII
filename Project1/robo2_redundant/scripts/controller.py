#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()
        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)


        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        self.x_end_effector_pub = rospy.Publisher('/x_end_effector', Float64, queue_size=100)
        self.y_end_effector_pub = rospy.Publisher('/y_end_effector', Float64, queue_size=100)
        self.z_end_effector_pub = rospy.Publisher('/z_end_effector', Float64, queue_size=100)

        self.x_dot_end_effector_pub = rospy.Publisher('/x_dot_end_effector', Float64, queue_size=100)
        self.y_dot_end_effector_pub = rospy.Publisher('/y_dot_end_effector', Float64, queue_size=100)
        self.z_dot_end_effector_pub = rospy.Publisher('/z_dot_end_effector', Float64, queue_size=100)

        self.x_acc_end_effector_pub = rospy.Publisher('/x_acceleration_end_effector', Float64, queue_size=100)
        self.y_acc_end_effector_pub = rospy.Publisher('/y_acceleration_end_effector', Float64, queue_size=100)
        self.z_acc_end_effector_pub = rospy.Publisher('/z_acceleration_end_effector', Float64, queue_size=100)

        self.x_error_pub = rospy.Publisher('/x_error', Float64, queue_size=100)
        self.y_error_pub = rospy.Publisher('/y_error', Float64, queue_size=100)
        self.z_error_pub = rospy.Publisher('/z_error', Float64, queue_size=100)

        self.y_distance_joint3_green = rospy.Publisher('/y_axis_distance_joint_3_green_obstacle', Float64, queue_size=100)
        self.y_distance_joint3_red = rospy.Publisher('/y_axis_distance_joint_3_red_obstacle', Float64, queue_size=100)
        self.y_distance_joint4_green = rospy.Publisher('/y_axis_distance_joint_4_green_obstacle', Float64, queue_size=100)
        self.y_distance_joint4_red = rospy.Publisher('/y_axis_distance_joint_4_red_obstacle', Float64, queue_size=100)

        self.distance_joint3_green = rospy.Publisher('/distance_joint_3_green_obstacle', Float64, queue_size=100)
        self.distance_joint3_red = rospy.Publisher('/distance_joint_3_red_obstacle', Float64, queue_size=100)
        self.distance_joint4_green = rospy.Publisher('/distance_joint_4_green_obstacle', Float64, queue_size=100)
        self.distance_joint4_red = rospy.Publisher('/distance_joint_4_red_obstacle', Float64, queue_size=100)

        self.y_red = rospy.Publisher('/red_obstacle_y_position', Float64, queue_size=100)
        self.y_green = rospy.Publisher('/green_obstacle_y_position', Float64, queue_size=100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def trajectory(self, t0, tf, yA, yB, g):
        t1 = t0 + (tf - t0) * 0.1
        t2 = t0 + (tf - t0) * 0.9

        a = np.array([[1, t0, t0**2, t0**3, t0**4, t0**5, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 1, 2*t0, 3 * t0**2, 4 * t0**3, 5 * t0**4, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3, 0, 0, 0, 0, 0, 0, 0, 0],
                    [1, t1, t1**2, t1**3, t1**4, t1**5, -1, - t1, 0, 0, 0, 0, 0, 0],
                    [0, 1, 2*t1, 3*t1**2, 4*t1**3, 5*t1**4, 0, -1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 2, 6*t1, 12*t1**2, 20*t1**3, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, -1, -t2, 1, t2, t2**2, t2**3, t2**4, t2**5],
                    [0, 0, 0, 0, 0, 0, 0, -1, 0, 1, 2 * t2, 3 * t2**2, 4*t2**3, 5*t2**4],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 6*t2, 12*t2**2, 20*t2**3],
                    [0, 0, 0, 0, 0, 0, 0, 0, 1, tf, tf**2, tf**3, tf**4, tf**5],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2*tf, 3 * tf**2, 4 * tf**3, 5 * tf**4],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 6*tf, 12*tf**2, 20*tf**3],
                    [-1, -t0, -t0**2, -t0**3, -t0**4, -t0**5, 1, t1 - 0.5*t1 + 0.5*t0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, -1, -t2-0.5*t1+0.5*t0, 1, tf, tf**2, tf**3, tf**4, tf**5]])

        b = np.array([yA, 0, g, 0, 0, 0, 0, 0, 0, yB, 0, -g, 0, 0])
        params = np.linalg.inv(a).dot(b)
        return params


    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        x_A = 0.6043
        x_B = 0.6043
        y_A = -0.2
        y_B = 0.2
        z_A = 0.1508
        z_B = 0.1508
        self.x_OBSTACLE1 = 0.3
        self.y_OBSTACLE1 = 0.2
        self.x_OBSTACLE2 = 0.3
        self.y_OBSTACLE2 = -0.2
        rostime_starttime = rospy.get_rostime()
        starttime = rostime_starttime.to_nsec()

        p_prev = np.zeros(3)

        while not rospy.is_shutdown():

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            self.A01 = self.kinematics.tf_A01(self.joint_angpos)
            self.A02 = self.kinematics.tf_A02(self.joint_angpos)
            self.A03 = self.kinematics.tf_A03(self.joint_angpos)
            self.A04 = self.kinematics.tf_A04(self.joint_angpos)
            self.A05 = self.kinematics.tf_A05(self.joint_angpos)
            self.A06 = self.kinematics.tf_A06(self.joint_angpos)
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)

            self.l2 = self.kinematics.l2            
            self.l3 = self.kinematics.l3
            self.l4 = self.kinematics.l4
            self.theta1 = self.kinematics.theta1

            s1 = np.sin(self.joint_angpos[0])
            c1 = np.cos(self.joint_angpos[0])
            s2 = np.sin(self.joint_angpos[1])
            c2 = np.cos(self.joint_angpos[1])
            s3 = np.sin(self.joint_angpos[2])
            c3 = np.cos(self.joint_angpos[2])
            s4 = np.sin(self.joint_angpos[3])
            c4 = np.cos(self.joint_angpos[3])
            s5 = np.sin(self.joint_angpos[4])
            c5 = np.cos(self.joint_angpos[4])


            #task 1

            p1d_x = x_A
            p1d_x_dot = 0
            acc_x = 0            

            time =  (time_now - starttime)/1e9
            period = 1
            A_to_B = True            
            if time < period:
            	A_to_B = False
            	if time < period * 0.1:
            		params = self.trajectory(0, period, 0, y_B, 1)
            		p1d_y = params[0] + params[1] * time + params[2] * time ** 2 + params[3] * time**3 + params[4] * time ** 4 + params[5] * time ** 5
            		p1d_y_dot =  params[1] + 2 * params[2] * time + 3 * params[3] * time**2 + 4 * params[4] * time ** 3 + 5 * params[5] * time ** 4
            		acc_y = 2 * params[2] + 6 * params[3] * time + 12 * params[4] * time ** 2 + 20 * params[5] * time ** 3
            	elif time < period * 0.9:
            		params = self.trajectory(0, period, 0, y_B, 1)
            		p1d_y = params[6] + params[7] * time
            		p1d_y_dot = params[7]
            		acc_y = 0
            	else:
            		params = self.trajectory(0, period, 0, y_B, 1)
            		p1d_y = params[8] + params[9] * time + params[10] * time ** 2 + params[11] * time**3 + params[12] * time ** 4 + params[13] * time ** 5
            		p1d_y_dot =  params[9] + 2 * params[10] * time + 3 * params[11] * time**2 + 4 * params[12] * time ** 3 + 5 * params[13] * time ** 4
            		acc_y = 2 * params[10] + 6 * params[11] * time + 12 * params[12] * time ** 2 + 20 * params[13] * time ** 3

            else:
            	t = time/period
            	start = np.floor(t)
            	if (start % 2 == 0):
            		t = time - start
            	else:
            		t = t - start + period
            	if t < period * 0.1:
            		params = self.trajectory(start, start + period, y_A, y_B, 1)
            		p1d_y = params[0] + params[1] * time + params[2] * time ** 2 + params[3] * time**3 + params[4] * time ** 4 + params[5] * time ** 5                
            		p1d_y_dot =  params[1] + 2 * params[2] * time + 3 * params[3] * time**2 + 4 * params[4] * time ** 3 + 5 * params[5] * time ** 4
            		acc_y = 2 * params[2] + 6 * params[3] * time + 12 * params[4] * time ** 2 + 20 * params[5] * time ** 3
            		A_to_B = False
            	elif  t < period * 0.9:
            		params = self.trajectory(start, start + period, y_A, y_B, 1)
            		p1d_y = params[6] + params[7] * time         
            		p1d_y_dot = params[7]
            		acc_y = 0
            		A_to_B = False
            	elif t < period:
            		params = self.trajectory(start, start + period, y_A, y_B, 1)
            		p1d_y = params[8] + params[9] * time + params[10] * time ** 2 + params[11] * time**3 + params[12] * time ** 4 + params[13] * time ** 5             
            		p1d_y_dot =  params[9] + 2 * params[10] * time + 3 * params[11] * time**2 + 4 * params[12] * time ** 3 + 5 * params[13] * time ** 4
            		acc_y = 2 * params[10] + 6 * params[11] * time + 12 * params[12] * time ** 2 + 20 * params[13] * time ** 3
            		A_to_B = False
            	elif t < period * 1.1:
            		params = self.trajectory(start, start + period, y_B, y_A, -1)
            		p1d_y = params[0] + params[1] * time + params[2] * time ** 2 + params[3] * time**3 + params[4] * time ** 4 + params[5] * time ** 5             
            		p1d_y_dot =  params[1] + 2 * params[2] * time + 3 * params[3] * time**2 + 4 * params[4] * time ** 3 + 5 * params[5] * time ** 4
            		acc_y = 2 * params[2] + 6 * params[3] * time + 12 * params[4] * time ** 2 + 20 * params[5] * time ** 3
            		A_to_B = True
            	elif t < period * 1.9:
            		params = self.trajectory(start, start + period, y_B, y_A, -1)
            		p1d_y = params[6] + params[7] * time               
            		p1d_y_dot = params[7]
            		acc_y = 0
            		A_to_B = True
            	else:
            		params = self.trajectory(start, start + period, y_B, y_A, -1)
            		p1d_y = params[8] + params[9] * time + params[10] * time ** 2 + params[11] * time**3 + params[12] * time ** 4 + params[13] * time ** 5               
            		p1d_y_dot =  params[9] + 2 * params[10] * time + 3 * params[11] * time**2 + 4 * params[12] * time ** 3 + 5 * params[13] * time ** 4
            		acc_y = 2 * params[10] + 6 * params[11] * time + 12 * params[12] * time ** 2 + 20 * params[13] * time ** 3
            		A_to_B = True


            p1d_z = z_A
            p1d_z_dot = 0
            acc_z = 0

            p1d = np.matrix([[p1d_x],\
                             [p1d_y],\
                             [p1d_z]])

            p1d_dot = np.matrix([[p1d_x_dot],\
                                 [p1d_y_dot],\
                                 [p1d_z_dot]])
            
            p1d_real = np.matrix([[self.A07[0, 3]],\
                                 [self.A07[1, 3]],\
                                 [self.A07[2, 3]]])

            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)
            # pseudoinverse jacobian
            pinvJ = pinv(J)

            K1 = 100
            self.task1 = np.dot(pinvJ, p1d_dot + K1 * (p1d- p1d_real))

            #task 2

            middle_obstacle = (self.model_states.pose[1].position.y + self.model_states.pose[2].position.y) / 2

            if A_to_B:
            	middle_obstacle = middle_obstacle + 0.075
            else:
            	middle_obstacle = middle_obstacle - 0.05

            self.y_OBSTACLE2 = self.model_states.pose[1].position.y
            self.y_OBSTACLE1 = self.model_states.pose[2].position.y
            self.x_OBSTACLE2 = self.model_states.pose[1].position.x
            self.x_OBSTACLE1 = self.model_states.pose[2].position.x

            K2 = 10
            K3 = 15
            Kc = 10
            dist3 = (1/2) * Kc * ((self.A03[1,3] - middle_obstacle) ** 2)
            dist4 = (1/2) * Kc * ((self.A04[1,3] - middle_obstacle) ** 2)
            
              
            dist3_dot = np.zeros((7,1))
            dist3_dot[0] = -Kc * (self.A03[1,3] - self.y_OBSTACLE2) * self.l2 * c1 * s2
            dist3_dot[1] = -Kc * (self.A03[1,3] - self.y_OBSTACLE2) * self.l2 * c2 * s1
            dist3_dot[2] = 0
            dist3_dot[3] = 0
            dist3_dot[4] = 0
            dist3_dot[5] = 0
            dist3_dot[6] = 0                   

            dist4_dot = np.zeros((7,1))
            dist4_dot[0] = -Kc * (self.A04[1,3] - self.y_OBSTACLE1) * (self.l2 * c1 * s2 - self.l3 * (s1 * s3 - c1 * c2 * c3))
            dist4_dot[1] = -Kc * (self.A04[1,3] - self.y_OBSTACLE1) * (self.l2 * c2 * s1 - self.l3 * c3 * s1 * s2)
            dist4_dot[2] = -Kc * (self.A04[1,3] - self.y_OBSTACLE1) * (self.l3 * (c1 * c3 - c2 * s1 * s3))
            dist4_dot[3] = 0
            dist4_dot[4] = 0
            dist4_dot[5] = 0
            dist4_dot[6] = 0

            I = np.eye(7)

            self.task2 =  np.dot( I - np.dot(pinvJ, J) , K2 * dist3_dot + K3 * dist4_dot) 

            maximum = max(dist3, dist4)
            if (maximum >= 0.03):
            	for i in range(7):
            		self.joint_angvel[i] = self.task1[i, 0] + self.task2[i, 0]
            else:
            	for i in range(7):
            		self.joint_angvel[i] = self.task1[i, 0]


            # Convertion to angular position after integrating the angular speed in time
            # Calculate time interval
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )

            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            self.x_end_effector_pub.publish(self.A07[0, 3])
            self.y_end_effector_pub.publish(self.A07[1, 3])
            self.z_end_effector_pub.publish(self.A07[2, 3])

            self.x_dot_end_effector_pub.publish(p1d_x_dot)
            self.y_dot_end_effector_pub.publish(p1d_y_dot)
            self.z_dot_end_effector_pub.publish(p1d_z_dot)

            self.x_acc_end_effector_pub.publish(acc_x)
            self.y_acc_end_effector_pub.publish(acc_y)
            self.z_acc_end_effector_pub.publish(acc_z)

            self.x_error_pub.publish(self.A07[0, 3] - p1d_x)
            self.y_error_pub.publish(self.A07[1, 3] - p1d_y)
            self.z_error_pub.publish(self.A07[2, 3] - p1d_z)

            self.y_distance_joint3_green.publish(self.A03[1, 3] - self.y_OBSTACLE2)
            self.y_distance_joint3_red.publish(self.A03[1, 3] - self.y_OBSTACLE1)
            self.y_distance_joint4_green.publish(self.A04[1, 3] - self.y_OBSTACLE2)
            self.y_distance_joint4_red.publish(self.A04[1, 3] - self.y_OBSTACLE1)
            
            self.distance_joint3_green.publish(np.sqrt((self.A03[0, 3] - self.y_OBSTACLE2) ** 2 + (self.A03[1, 3] - self.x_OBSTACLE2) ** 2))
            self.distance_joint3_red.publish(np.sqrt((self.A03[0, 3] - self.y_OBSTACLE1) ** 2 + (self.A03[1, 3] - self.x_OBSTACLE1) ** 2))
            self.distance_joint4_green.publish(np.sqrt((self.A04[0, 3] - self.y_OBSTACLE2) ** 2 + (self.A04[1, 3] - self.x_OBSTACLE2) ** 2))
            self.distance_joint4_red.publish(np.sqrt((self.A04[0, 3] - self.y_OBSTACLE1) ** 2 + (self.A04[1, 3] - self.x_OBSTACLE1) ** 2))

            self.y_red.publish(self.y_OBSTACLE1)
            self.y_green.publish(self.y_OBSTACLE2)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
