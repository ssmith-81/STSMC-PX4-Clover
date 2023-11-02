#!/usr/bin/python

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
import tf

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

#from clover import srv
from time import sleep
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('Helix')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Release service to allow for complex trajectory publishing i.e stopping navigate service publishing because you dont want two sources of publishing at the same time.
release = rospy.ServiceProxy('simple_offboard/release', Trigger)

PI_2 = math.pi/2

class fcuModes:

	def __init__(self):
		pass

	def setArm(self):
		rospy.wait_for_service('mavros/cmd/arming')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
			armService(True)
		except rospy.ServiceException as e:
			print ("Service arming call failed: %s")

	def setOffboardMode(self):
		rospy.wait_for_service('mavros/set_mode')
		try:
			flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
			flightModeService(custom_mode='OFFBOARD')
		except rospy.ServiceException as e:
			print ("service set_mode call failed: %s. Offboard Mode could not be set.")

class clover:

	def __init__(self, FLIGHT_ALTITUDE = 1.0, RATE = 50, RADIUS = 2, CYCLE_S = 27): # rate = 50hz radius = 5m cycle_s = 25
        
 		
 		# Publisher which will publish to the topic '/mavros/setpoint_velocity/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped, queue_size=10)

		 # global static variables
		self.FLIGHT_ALTITUDE = FLIGHT_ALTITUDE          # fgdf
		self.RATE            = RATE                     # loop rate hz
		self.RADIUS          = RADIUS                   # radius of figure 8 in meters
		self.CYCLE_S         = CYCLE_S                  # time to complete one figure 8 cycle in seconds
		self.STEPS           = int( self.CYCLE_S * self.RATE )

		
		# Publisher which will publish to the topic '/mavros/setpoint_raw/local'.
		self.publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
		
		# Subscribe to drone state
		self.state = rospy.Subscriber('mavros/state', State, self.updateState)

		self.current_state = State()
		self.rate = rospy.Rate(20)

	def updateState(self, msg):
		self.current_state = msg

	def main(self):#,x=0, y=0, z=2, yaw = float('nan'), speed=1, frame_id ='',auto_arm = True,tolerance = 0.2):
		i    = 0
		dt   = 1.0/self.RATE
		dadt = math.pi*2 / self.CYCLE_S # first derivative of angle with respect to time
		r    = self.RADIUS
		path = []
		

		# Wait for 5 seconds
		rospy.sleep(3)
		
		# Takeoff with Clovers navigate function
		navigate(x=0, y=0, z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = 'body', auto_arm = True)
		tolerance =0.2
		
		# Base the flight on time and not performance so I can plot against one another in matlab (want aligned time) --> Only needed for non wind case because I include takeoff in that analysis
		# Base takeoff on performance when including wind because t aeks a while to find the starting point and reject wind
		#while not rospy.is_shutdown():
		#	telem = get_telemetry(frame_id = 'navigate_target')
		#	if math.sqrt(telem.x**2 + telem.y**2 + telem.z**2) < tolerance:
		#		break
		#	rospy.sleep(0.2)

		# Wait for 11 seconds
		rospy.sleep(11)

		#k = get_telemetry()
		#print(k.mode)
		
		rospy.loginfo('start figure8')
		PI=math.pi
		start = get_telemetry()
		start_stamp = rospy.get_rostime()
		
		# create random time array
		t = np.arange(0,self.STEPS,1)
		
		posx = [1]*len(t)
		posy = [1]*len(t)
		posz = [1]*len(t)
		velx = [1]*len(t)
		vely = [1]*len(t)
		velz = [1]*len(t)
		afx = [1]*len(t)
		afy = [1]*len(t)
		afz = [1]*len(t)
		yawc = [1]*len(t)
		pitchc = [1]*len(t)
		rollc = [1]*len(t)
		yaw_ratec = [1]*len(t)
		
		for i in range(0, self.STEPS):
		
			# calculate the parameter a which is an angle sweeping from -pi/2 to 3pi/2
			# through the curve
			a = (-math.pi/2) + i*(math.pi*2/self.STEPS)
			c = math.cos(a)
			c2a = math.cos(2.0*a)
			c4a = math.cos(4.0*a)
			c2am3 = c2a-3.0
			c2am3_cubed = c2am3*c2am3*c2am3
			s = math.sin(a)
			cc = c*c
			ss = s*s
			sspo = (s*s)+1.0 # sin squared plus one
			ssmo = (s*s)-1.0 # sin squared minus one
			sspos = sspo*sspo
			
			 # Position
			# https:#www.wolframalpha.com/input/?i=%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posx[i] = -(r*c*s) / sspo
			# https:#www.wolframalpha.com/input/?i=%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29
			posy[i] =  (r*c)   / sspo
			posz[i] =  self.FLIGHT_ALTITUDE

			# Velocity
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			velx[i] =   dadt*r* ( ss*ss + ss + (ssmo*cc) ) / sspos
			# https:#www.wolframalpha.com/input/?i=derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			vely[i] =  -dadt*r* s*( ss + 2.0*cc + 1.0 )    / sspos
			velz[i] =  0.0

			# Acceleration
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28-r*cos%28a%29*sin%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afx[i] =  -dadt*dadt*8.0*r*s*c*((3.0*c2a) + 7.0)/ c2am3_cubed
			# https:#www.wolframalpha.com/input/?i=second+derivative+of+%28r*cos%28a%29%29%2F%28%28sin%28a%29%5E2%29%2B1%29+wrt+a
			afy[i] =  dadt*dadt*r*c*((44.0*c2a) + c4a - 21.0) / c2am3_cubed
			afz[i] =  0.0

			# calculate yaw as direction of velocity
			yawc[i] = math.atan2(vely[i], velx[i])
			
			# calculate Pitch and Roll angles, if publishing acceleration isnt possible could send these low level commands
			pitchc[i] = math.asin(afx[i]/9.81)
			rollc[i] = math.atan2(afy[i], afz[i])
		
		# calculate yaw_rate by dirty differentiating yaw
		for i in range(0, self.STEPS):
			next = yawc[(i+1)%self.STEPS] # 401%400 = 1 --> used for this reason (when it goes over place 400 or whatever STEPS is)
			curr = yawc[i]
      			# account for wrap around +- PI
			if((next-curr) < -math.pi):
				next = next + math.pi*2
			if((next-curr) >  math.pi):
				next = next - math.pi*2
			yaw_ratec[i] = (next-curr)/dt
		
		# Define 
		target = PositionTarget()
		rr = rospy.Rate(self.RATE)
		k=0
		release() # stop navigate service from publishing
		while not rospy.is_shutdown():
		
			target.header.frame_id = 'aruco_map'
			
			target.coordinate_frame = 1 #MAV_FRAME_LOCAL_NED  # =1
			
			target.type_mask = 0 # Use everything!
			#target.type_mask =  3576 # Use only position #POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE | POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |POSITION_TARGET_TYPEMASK_AZ_IGNORE | POSITION_TARGET_TYPEMASK_FORCE_IGNORE | POSITION_TARGET_TYPEMASK_YAW_IGNORE | POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE # = 3576
			#target.type_mask =  3520 # Use position and velocity
			#target.type_mask =  3072 # Use position, velocity, and acceleration
			#target.type_mask =  2048 # Use position, velocity, acceleration, and yaw

			# Gather position for publishing
			target.position.x = posx[k] +0.15
			target.position.y = posy[k]
			target.position.z = posz[k]
			
			# Gather velocity for publishing
			target.velocity.x = velx[k]
			target.velocity.y = vely[k]
			target.velocity.z = velz[k]
			
			# Gather acceleration for publishing
			target.acceleration_or_force.x = afx[k]
			target.acceleration_or_force.y = afy[k]
			target.acceleration_or_force.z = afz[k]
			
			# Gather yaw for publishing
			target.yaw = yawc[k]
			
			# Gather yaw rate for publishing
			target.yaw_rate = yaw_ratec[k]
			
			
			self.publisher.publish(target)
			
			
		
			#set_position(x=posx[k], y=posy[k], z=posz[k],frame_id='aruco_map')
			#set_velocity(vx=velx[k], vy=vely[k], vz=velz[k],frame_id='aruco_map')#,yaw = yawc[i]) 
			#set_attitude(yaw = yawc[k],pitch = pitchc[k], roll = rollc[k], thrust = float('nan'),frame_id='aruco_map')
			#set_rates(yaw_rate = yaw_ratec[k],thrust = float('nan'))
		
		
			
			k = k+1
			if k >= self.STEPS: 
				navigate(x=0, y=0, z=self.FLIGHT_ALTITUDE, yaw=float('nan'), speed=0.5, frame_id = 'aruco_map')
				rospy.sleep(5)
				break
			rr.sleep()

		# Wait for 5 seconds
		rospy.sleep(2)
		# Perform landing
		
		res = land()
		 
		if res.success:
			print('Drone is landing')

	# If we press control + C, the node will stop.
		rospy.sleep(6)
		# debug section
		plt.plot(t,velx)
		plt.plot(t,posx)
		plt.plot(t,afx)
		#plt.plot(t,yawc)
		#plt.plot(t,yaw_ratec)
		#plt.show()
		
		rospy.spin()

if __name__ == '__main__':
	try:
		#setArm()
		q=clover()
		q.main()#x=0,y=0,z=1,frame_id='aruco_map')
		
	except rospy.ROSInterruptException:
		pass

