#!/usr/bin/env python
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from std_msgs.msg import Float64


# Sonars:
sonarF  = 0.0
sonarFL = 0.0
sonarFR = 0.0
sonarL  = 0.0
sonarR  = 0.0

# IMU:
imuRoll  = 0.0 		# orientation
imuPitch = 0.0
imuYaw   = 0.0
imuAngVelX = 0.0 	# angular velocity
imuAngVelY = 0.0
imuAngVelZ = 0.0
imuLinAccX = 0.0 	# linear acceleration
imuLinAccY = 0.0
imuLinAccZ = 0.0

# 0:output / 1:prediction / 2:measurement
x  = np.array([0.0,0.0,0.0]) 	
y  = np.array([0.0,0.0,0.0])
th = np.array([-8%np.pi,0.0,0.0])
vx = np.array([0.0,0.0,0.0])
ax = 0.0
wz = 0.0
dt = 0.1			# Rate = 10Hz
P = np.zeros(4)



nothing_chosen = False
boolF  = True
boolFL = True
boolFR = True
boolL  = True
boolR  = True
once   = True
x_predicted = 0
y_predicted = 0

x_0  = []
y_0  = []
x_1 = []
y_1 = []
x_2 = []
y_2 = []
time    = []
t = 0

x_coord = rospy.Publisher('/coordinate_x', Float64, queue_size = 100)
x_pred = rospy.Publisher('/coordinate_x_pred', Float64, queue_size = 100)
x_meas = rospy.Publisher('/coordinate_x_meas', Float64, queue_size = 100)
y_coord = rospy.Publisher('/coordinate_y', Float64, queue_size = 100)
y_pred = rospy.Publisher('/coordinate_y_pred', Float64, queue_size = 100)
y_meas = rospy.Publisher('/coordinate_y_meas', Float64, queue_size = 100)
theta_coord = rospy.Publisher('/coordinate_theta', Float64, queue_size = 100)

def estimate_xy():
	global sonarF,sonarL,sonarR,sonarFR,sonarFL,x,y,th,boolF,boolFL,boolFR,boolR,boolL,x_predicted,y_predicted,nothing_chosen
	limit = 1.5
	x0 = 2
	y0 = 2

	l1 = 0.018
	l3 = 0.1
	l4 = 0.2 

	phi = math.atan(l3 / l4)

	l_diag = np.sqrt(l3 ** 2 + l4 ** 2) - l1 * np.cos(np.pi/2 - phi)

	angle_L = th[0] - np.pi/2
	angle_R = th[0] + np.pi/2
	angle_FL = th[0] - phi
	angle_FR = th[0] + phi

	theta = abs(th[0])	

	if (sonarF <= limit and boolF):		#front sonar

		if np.pi/4 < th[0] <= 3*np.pi/4:	#wall 1 
			h = y[2]
			y[2] = y0 - (sonarF + l4) * np.sin(theta) + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 1

		elif -np.pi/4 < th[0] <= np.pi/4:	#wall 2 
			h = x[2]
			x[2] = x0 - (sonarF + l4) * np.cos(theta) + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 2

		elif -3*np.pi/4 < th[0] <= -np.pi/4:	#wall 3
			h = y[2]
			y[2] = (sonarF + l4) * np.cos(theta - np.pi/2) - y0 +y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 3
		else:					#wall 4 
			h = x[2]
			x[2] = (sonarF + l4) * np.cos(np.pi - theta)  - x0 + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 4	   

		boolF = False

	elif (sonarL <= limit and boolL):	#left sonar


		if np.pi/4 < angle_L <= 3*np.pi/4:	#wall 1" 
			h = y[2]
			y[2] = y0 - (sonarL + l3) * np.sin(theta - np.pi/2) + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 1

		elif -np.pi/4 < angle_L  <= np.pi/4:	#wall 2
			h = x[2]
			x[2] = x0 - (sonarL + l3) * np.cos(np.pi/2 - theta) + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 2

		elif -3*np.pi/4 < angle_L <= -np.pi/4:	#wall 3
			h = y[2]
			y[2] = (sonarL + l3) * np.cos(theta) - y0 + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 3
		else:					#wall 4 
			h = x[2]
			x[2] = (sonarL + l3) * np.cos(theta - np.pi/2) - x0 + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 4	   

		boolL = False
		
	elif (sonarR <= limit and boolR):	#right sonar

		if np.pi/4 < angle_R <= 3*np.pi/4:	#wall 1 
			h = y[2]
			y[2] = y0 - (sonarR + l3) * np.sin(np.pi/2 - theta) + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 1

		elif -np.pi/4 < angle_R  <= np.pi/4: 	 #wall 2
			h = x[2]
			x[2] = x0 - (sonarR + l3) * np.cos(theta - np.pi/2) + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 2

		elif -3*np.pi/4 < angle_R <= -np.pi/4:	#wall 3 
			h = y[2]
			y[2] = (sonarR + l3) * np.cos(np.pi - theta) - y0 +y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 3
		else:					#wall 4 
			h = x[2]
			x[2] = (sonarR + l3) * np.cos(np.pi/2 - theta) - x0 + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 4	
   
		boolR = False
		
	elif (sonarFL <= limit and boolFL):	#front-left sonar

		if np.pi/4 < angle_FL <= 3*np.pi/4:	#wall 1 
			h = y[2]
			y[2] = y0 - (sonarFL + l_diag) * np.cos(np.pi/2 - theta + phi) + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 1

		elif -np.pi/4 < angle_FL <= np.pi/4:	#wall 2
			h = x[2]
			if th[0] >= 0:
				x[2] = x0 - (sonarFL + l_diag) * np.cos(theta - phi)
			else:
				x[2] = x0 - (sonarFL + l_diag) * np.cos(theta + phi)
			x_predicted = x_predicted + 1
			chosen_wall = 2

		elif -3*np.pi/4 < angle_FL <= -np.pi/4:	#wall 3
			h = y[2]
			y[2] = (sonarFL + l_diag) * np.cos(np.pi/2 - theta - phi) - y0 +y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 3
		else:					#wall 4
			h = x[2]
			if th[0] >= 0:
				x[2] = (sonarFL + l_diag) * np.cos(np.pi - theta + phi) - x0 + x[2]
			else:
				x[2] = (sonarFL + l_diag) * np.cos(np.pi - theta - phi) - x0 + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 4	 
  
		boolFL = False
	
	elif (sonarFR <= limit and boolFR):	#front-right sonar

		if np.pi/4 < angle_FR <= 3*np.pi/4:	#wall 1
			h = y[2]
			y[2] = y0 - (sonarFR + l_diag) * np.cos(np.pi/2 - theta - phi) + y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 1

		elif -np.pi/4 < angle_FR <= np.pi/4:	#wall 2
			h = x[2]
			if th[0] >= 0:
				x[2] = x0 - (sonarFR + l_diag) * np.cos(theta + phi)
			else:
				x[2] = x0 - (sonarFR + l_diag) * np.cos(theta - phi)
			x_predicted = x_predicted + 1
			chosen_wall = 2	

		elif -3*np.pi/4 < angle_FR <= -np.pi/4:	#wall 3
			h = y[2]
			y[2] = (sonarFR + l_diag) * np.cos(np.pi/2 - theta + phi) - y0 +y[2]
			y_predicted = y_predicted + 1
			chosen_wall = 3
		else:					#wall 4 
			h = x[2]
			if th[0] >= 0:
				x[2] = (sonarFR + l_diag) * np.cos(np.pi - theta - phi) - x0 + x[2]
			else:
				x[2] = (sonarFR + l_diag) * np.cos(np.pi - theta + phi) - x0 + x[2]
			x_predicted = x_predicted + 1
			chosen_wall = 4
	   
		boolFR = False

		
	else:
		nothing_chosen = True
		return


	

def control_input():
	global imuLinAccX, imuAngVelZ, ax, wz
	ax = imuLinAccX
	wz = imuAngVelZ

def prediction():
	global x,y,th,vx,P,dt,ax,wz,P

	dax = 0.002
	dwz = 0.002

	# estimated errors
	dth = dwz*dt
	dvx = dax*dt
	dx  = 1.0/2*dax*np.cos(th[0])*dt*dt
	dy  = 1.0/2*dax*np.sin(th[0])*dt*dt

	# prediction equations
	x[1]  = x[0]  + vx[0]*np.cos(th[0])*dt + 1/2*ax*np.cos(th[0])*dt*dt
	y[1]  = y[0]  + vx[0]*np.sin(th[0])*dt + 1/2*ax*np.sin(th[0])*dt*dt
	vx[1] = vx[0] +                  ax*dt
	th[1] = th[0] +			 wz*dt

	x_pred.publish(x[1])
	y_pred.publish(y[1])
	

	# prediction filtering
	if (x[1] > x[0]+0.02):   x[1] = x[0]+0.02
	elif (x[1] < x[0]-0.02): x[1] = x[0]-0.02
	if (y[1] > y[0]+0.02):   y[1] = y[0]+0.02
	elif (y[1] < y[0]-0.02): y[1] = y[0]-0.02
	if vx[1]>0.25:  vx[1] = 0.25
	if vx[1]<0.02: vx[1] = 0.0

	Cw = np.array([[dx**2 ,dx*dy ,0.0   ,dx*dvx],
		       [dx*dy ,dy**2 ,0.0   ,dy*dvx],
		       [0.0   ,0.0   ,dth**2,0.0   ],
	               [dx*dvx,dy*dvx,0.0   ,dvx**2]])

	Al = np.array([[1.0,0.0,-vx[0]*np.sin(th[0])*dt-1/2*ax*np.sin(th[0])*dt*dt,np.cos(th[0])*dt],
		       [0.0,1.0,+vx[0]*np.cos(th[0])*dt+1/2*ax*np.cos(th[0])*dt*dt,np.sin(th[0])*dt],
		       [0.0,0.0,1.0                                               ,0.0             ],
		       [0.0,0.0,0.0                                               ,1.0             ]])



	P = (np.matmul(np.matmul(Al,P),np.transpose(Al)) + Cw)
	

def measurement():
	global x,y,th,vx,imuYaw,boolF,boolFL,boolL,boolR,boolFR,x_predicted,y_predicted,nothing_chosen

	boolF  = True
	boolFL = True
	boolFR = True
	boolL  = True
	boolR  = True
	nothing_chosen = False
	x_predicted = 0
	y_predicted = 0

	x[2] = 0
	y[2] = 0
	while not nothing_chosen:
		estimate_xy()

	# fuse the measurements by sonars by taking the mean of the measurements
	if (x_predicted==0): x[2] = x[1]
	else: 		     x[2] = x[2]/x_predicted	
	if (y_predicted==0): y[2] = y[1]
	else: 		     y[2] = y[2]/y_predicted
	
	x_meas.publish(x[2])
	y_meas.publish(y[2])
	#Measurement of theta
	th[2] = imuYaw

	vx[2] = vx[1]
	if (abs(wz)>0.1):
 		vx[2] = 0

	# measurement filtering
	if (x[2] > x[0]+0.1):   x[2] = x[0]+0.1
	elif (x[2] < x[0]-0.1): x[2] = x[0]-0.1
	if (y[2] > y[0]+0.1):   y[2] = y[0]+0.1
	elif (y[2] < y[0]-0.1): y[2] = y[0]-0.1

def kalman_filter():
	global x, y, th, vx, P, R, x_cord, y_cord

	R = np.array([[0.001**2, 0.0, 0.0, 0.0],
		      [0.0, 0.001**2, 0.0, 0.0],
		      [0.0, 0.0, 0.002**2, 0.0],
	              [0.0, 0.0, 0.0, 0.002**2]])


	H = np.array([[1.0, 0.0, 0.0, 0.0],
		      [0.0, 1.0, 0.0, 0.0],
		      [0.0, 0.0, 1.0, 0.0],
		      [0.0, 0.0, 0.0, 1.0]])

	temp = np.matmul(np.matmul(H,P),np.transpose(H))+R
	K = np.matmul(np.matmul(P,np.transpose(H)),inv(temp))
	P = np.matmul((np.eye(4)-np.matmul(K,H)),P)

	pred = np.array([x[1],y[1],th[1],vx[1]])
	meas = np.array([x[2],y[2],th[2],vx[2]])
	state = pred + np.matmul(K,(meas-np.matmul(H,pred)))

	x[0]  = state[0]
	y[0]  = state[1]
	th[0] = state[2]
	vx[0] = state[3]

	x_coord.publish(x[0])
	y_coord.publish(y[0])
	theta_coord.publish(th[0])

	print "Estimated x:        " + str(x[0])
	print "Estimated y:        " + str(y[0])
	print ("Estimated theta:" + str(th[0]))
	print ("EStimated velocity: " + str(vx[0]))
	print ("Time:" + str(t/10.0) +" sec")

def diagrams():
	global once,t,x,y,th,time,x_0,y_0,x_1,y_1,x_2,y_2
	
	t = t + 1
	time.append(0.1*t)
	x_0.append(x[0])
	y_0.append(y[0])
	x_1.append(x[1])
	y_1.append(y[1])
	x_2.append(x[2])
	y_2.append(y[2])
	
	if (t == 2000 and once):
		once = False
		fig, axs = plt.subplots(2, 2)
		axes = axs.flatten()

		axes[0].scatter(y_1,x_1, c = "green", label = 'predicted')
		axes[0].legend(loc = "upper left")

		axes[1].scatter(y_2,x_2, c = "blue", label = 'measurement')
		axes[1].legend(loc = "upper left")

		axes[2].scatter(y_0,x_0, c = "orange", label = 'estimated')
		axes[2].legend(loc = "upper left")
		
		axes[3].axis('off')

		plt.show()


def send_velocity():
	global x,y,th,vx,wz

    	control_input();
    	prediction();
	measurement();
	kalman_filter();
	diagrams()


def sonarFrontCallback(msg):
	global sonarF
    	sonarF = msg.range;
    	send_velocity()

def sonarFrontLeftCallback(msg):
	global sonarFL
    	sonarFL = msg.range

def sonarFrontRightCallback(msg):
	global sonarFR
    	sonarFR = msg.range

def sonarLeftCallback(msg):
	global sonarL
   	sonarL = msg.range

def sonarRightCallback(msg):
	global sonarR
    	sonarR = msg.range

def imuCallback(msg):
    	global imuYaw,imuAngVelZ,imuLinAccX
    	orientation_q = msg.orientation
    	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    	(imuRoll, imuPitch, imuYaw) = euler_from_quaternion (orientation_list)
	imuYaw = -imuYaw
    	# angular velocity
    	imuAngVelZ = msg.angular_velocity.z
    	# linear acceleration
    	imuLinAccX = msg.linear_acceleration.x

def localizer_py():
	rospy.init_node("localization_node")
    	rospy.Subscriber("/sensor/sonar_F", Range, sonarFrontCallback)
   	rospy.Subscriber("/sensor/sonar_FL", Range, sonarFrontLeftCallback)
    	rospy.Subscriber("/sensor/sonar_FR", Range, sonarFrontRightCallback)
    	rospy.Subscriber("/sensor/sonar_L", Range, sonarLeftCallback)
    	rospy.Subscriber("/sensor/sonar_R", Range, sonarRightCallback)
    	rospy.Subscriber("/imu", Imu, imuCallback)
	rate = rospy.Rate(10)
    	while not rospy.is_shutdown():
        	rospy.spin()

if __name__ == '__main__':
    	try:
        	localizer_py()
    	except rospy.ROSInterruptException: pass
