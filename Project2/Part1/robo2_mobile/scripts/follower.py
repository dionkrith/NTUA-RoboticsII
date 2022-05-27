#!/usr/bin/env python3

"""
Start ROS node to publish linear and angular velocities to mymobibot in order to perform wall following.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# from tf.transformations import euler_from_quaternion
# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

def quaternion_to_euler(w, x, y, z):
    """Converts quaternions with components w, x, y, z into a tuple (roll, pitch, yaw)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x**2 + y**2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.where(np.abs(sinp) >= 1, np.sign(sinp) * np.pi / 2, np.arcsin(sinp))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y**2 + z**2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class mymobibot_follower():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # linear and angular velocity
        self.velocity = Twist()
        # joints' states
        self.joint_states = JointState()
        # Sensors
        self.imu = Imu()
        self.imu_yaw = 0.0 # (-pi, pi]
        self.sonar_F = Range()
        self.sonar_FL = Range()
        self.sonar_FR = Range()
        self.sonar_L = Range()
        self.sonar_R = Range()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.velocity_pub = rospy.Publisher('/mymobibot/cmd_vel', Twist, queue_size=1)
        self.joint_states_sub = rospy.Subscriber('/mymobibot/joint_states', JointState, self.joint_states_callback, queue_size=1)
        # Sensors
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sonar_front_sub = rospy.Subscriber('/sensor/sonar_F', Range, self.sonar_front_callback, queue_size=1)
        self.sonar_frontleft_sub = rospy.Subscriber('/sensor/sonar_FL', Range, self.sonar_frontleft_callback, queue_size=1)
        self.sonar_frontright_sub = rospy.Subscriber('/sensor/sonar_FR', Range, self.sonar_frontright_callback, queue_size=1)
        self.sonar_left_sub = rospy.Subscriber('/sensor/sonar_L', Range, self.sonar_left_callback, queue_size=1)
        self.sonar_right_sub = rospy.Subscriber('/sensor/sonar_R', Range, self.sonar_right_callback, queue_size=1)

        self.l_error_pub = rospy.Publisher('/L_error', Float64, queue_size = 100)
        self.fl_error_pub = rospy.Publisher('/FL_error', Float64, queue_size = 100)

        self.f_sonar_pub = rospy.Publisher('/F_sonar', Float64, queue_size = 100)
        self.fl_sonar_pub = rospy.Publisher('/FL_sonar', Float64, queue_size = 100)
        self.l_sonar_pub = rospy.Publisher('/L_sonar', Float64, queue_size = 100)

        self.l_sonar_plus_c1_pub = rospy.Publisher('/L_sonar_plus_c1', Float64, queue_size = 100)
        self.fl_sonar_plus_c2_pub = rospy.Publisher('/FL_sonar_plus_c2', Float64, queue_size = 100)

        self.velocity_z_pub = rospy.Publisher('/angular_velocity_z', Float64, queue_size = 100)
        self.velocity_x_pub = rospy.Publisher('/linear_velocity_x', Float64, queue_size = 100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of the left wheel is stored in :: self.joint_states.position[0])
        # (e.g. the angular velocity of the right wheel is stored in :: self.joint_states.velocity[1])

    def imu_callback(self, msg):
        # ROS callback to get the /imu

        self.imu = msg
        # (e.g. the orientation of the robot wrt the global frome is stored in :: self.imu.orientation)
        # (e.g. the angular velocity of the robot wrt its frome is stored in :: self.imu.angular_velocity)
        # (e.g. the linear acceleration of the robot wrt its frome is stored in :: self.imu.linear_acceleration)

        #quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        #(roll, pitch, self.imu_yaw) = euler_from_quaternion(quaternion)
        (roll, pitch, self.imu_yaw) = quaternion_to_euler(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def sonar_front_callback(self, msg):
        # ROS callback to get the /sensor/sonar_F

        self.sonar_F = msg
        # (e.g. the distance from sonar_front to an obstacle is stored in :: self.sonar_F.range)

    def sonar_frontleft_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FL

        self.sonar_FL = msg
        # (e.g. the distance from sonar_frontleft to an obstacle is stored in :: self.sonar_FL.range)

    def sonar_frontright_callback(self, msg):
        # ROS callback to get the /sensor/sonar_FR

        self.sonar_FR = msg
        # (e.g. the distance from sonar_frontright to an obstacle is stored in :: self.sonar_FR.range)

    def sonar_left_callback(self, msg):
        # ROS callback to get the /sensor/sonar_L

        self.sonar_L = msg
        # (e.g. the distance from sonar_left to an obstacle is stored in :: self.sonar_L.range)

    def sonar_right_callback(self, msg):
        # ROS callback to get the /sensor/sonar_R

        self.sonar_R = msg
        # (e.g. the distance from sonar_right to an obstacle is stored in :: self.sonar_R.range)

    def publish(self):

        # set configuration
        self.velocity.linear.x = 0.0
        self.velocity.angular.z = 0.0
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")
        
        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
        
        #initial state 0 
        state = 0

        front_distance_threshold = 0.5
        d1 = 0.2
        d2 = 0.3
        c1 = 0.3
        c2 = 0.25
        max_z_velocity = 0.5  

        Kp = 30
        Kd = 5      

        acc_x = 1
        acc_z = 2
        self.velocity.linear.x = 0

        while not rospy.is_shutdown():


            # Calculate time interval (in case is needed)
            time_prev = time_now
            rostime_now = rospy.get_rostime()
            time_now = rostime_now.to_nsec()
            dt = (time_now - time_prev)/1e9

            if state == 0:
            	if self.velocity.linear.x == 0:
            		self.velocity.linear.x = 0.1
            	elif self.velocity.linear.x < 0.4:
            		#linear increase of linear velocity in x axis
            		self.velocity.linear.x = self.velocity.linear.x + acc_x * dt  
            	else:
            		self.velocity.linear.x = 0.4

            	self.velocity.angular.z = 0.0

            	sonar_front = self.sonar_F.range

            	if sonar_front < front_distance_threshold:
            		#change from state 0 to state 1            		
            		state = 1

            elif state == 1:
            	if self.velocity.linear.x > 0.1:
            		#linear decreas of linear velocity in x axis
            		self.velocity.linear.x = self.velocity.linear.x - acc_x * dt 
            	else:
            		self.velocity.linear.x = 0.1

            	if self.velocity.angular.z < 1.5:
            		#linear increase of angular velocity in z axis
            		self.velocity.angular.z = self.velocity.angular.z + acc_z * dt 
            	else:
            		self.velocity.angular.z = 1.5 #CW - positive angular velocity

            	sonar_front = self.sonar_F.range
            	sonar_left = self.sonar_L.range
            	sonar_front_left = self.sonar_FL.range

            	if sonar_left + c1 < sonar_front and sonar_front_left + c2 < sonar_front:
            		#change from state 1 to state 2
            		state = 2 
            		L_error = 0
            		FL_error = 0

            elif state == 2:

            	sonar_front = self.sonar_F.range
            	sonar_left = self.sonar_L.range
            	sonar_front_left = self.sonar_FL.range

            	FL_error = d2 - sonar_front_left
            	L_error = d1 - sonar_left

            	propotional = FL_error + L_error

            	#detivatives errors
            	FL_error_derivative = FL_error - FL_error_prev
            	L_error_derivative = L_error - L_error_prev

            	derivative = (FL_error_derivative + L_error_derivative) / dt
            	
            	if self.velocity.linear.x < 0.4:
            		self.velocity.linear.x = self.velocity.linear.x + acc_x * dt
            	else:
            		self.velocity.linear.x = 0.4

            	self.velocity.angular.z = min(max(Kp * propotional + Kd * derivative, -max_z_velocity), max_z_velocity)

            	if sonar_left + c1 >= sonar_front and sonar_front_left + c2 >= sonar_front:
            		#change from state 2 to state 1
            		state = 1

            if state == 2:
            	FL_error_prev = FL_error
            	L_error_prev = L_error

            # Publish the new joint's angular positions
            self.velocity_pub.publish(self.velocity)

            self.l_error_pub.publish(d1 - self.sonar_L.range)
            self.fl_error_pub.publish(d2 - self.sonar_FL.range)
            self.velocity_z_pub.publish(self.velocity.angular.z)
            self.velocity_x_pub.publish(self.velocity.linear.x)

            self.f_sonar_pub.publish(self.sonar_F.range)
            self.fl_sonar_pub.publish(self.sonar_FL.range)
            self.l_sonar_pub.publish(self.sonar_L.range)

            self.l_sonar_plus_c1_pub.publish(self.sonar_L.range + c1)
            self.fl_sonar_plus_c2_pub.publish(self.sonar_FL.range + c2)

            self.pub_rate.sleep()

    def turn_off(self):
        pass

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    follower = mymobibot_follower(rate)
    rospy.on_shutdown(follower.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        follower_py()
    except rospy.ROSInterruptException:
        pass
