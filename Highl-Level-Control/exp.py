# Information: https://clover.coex.tech/programming

import rospy
import math
from clover import srv
from gazebo_msgs.srv import ApplyBodyWrench
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Wrench
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

# Want wrench service to apply force
apply_force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)

def setAutoLandMode():
	rospy.wait_for_service('mavros/set_mode')
	try:
		flightModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
		flightModeService(custom_mode='AUTO.LAND')
	except rospy.ServiceException as e:
		print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)

wrench = Wrench() # Define class objecy
# Set the force and torque you want to apply:
wrench.force.x = 0
wrench.force.y = 0
wrench.force.z = -4.3 # maybe something in 3-4 range
wrench.torque.x = 0
wrench.torque.x = 0
wrench.torque.x = 0
# Set the duration (<0 sets for rest of simulation)
duration = rospy.Duration(-1)
rospy.sleep(2) # Wasnt here during experiment
print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1.2, speed = 0.5,frame_id='body', auto_arm=True)
#set_position(x=0,y=0,z=1,frame_id='body',auto_arm=True)
#set_velocity(vx=0,vy=0,vz=0.1,frame_id='body',auto_arm=True)
#set_position(auto_arm=True)

#telem = get_telemetry(frame_id='map')
#while telem.z < 1:
#	set_position(x=0,y=0,z=1,frame_id='body')
#	set_velocity(vx=0,vy=0,vz=0.2,frame_id='body')
#	telem = get_telemetry(frame_id='map')
	

# Wait for 5 seconds
rospy.sleep(14)

print('Fly forward 1 m')
navigate(x=0, y=0, z=0.5, speed = 0.4, frame_id='aruco_map')
#set_position(x=1,y=0,z=1,frame_id='aruco_map')

# Wait for 5 seconds
rospy.sleep(6)

apply_force(body_name = 'clover::base_link',reference_frame='clover::base_link',wrench = wrench,duration = duration)


rospy.sleep(9)

#navigate(x=0,y=0,z=1,frame_id='aruco_map')

navigate(x=1,y=1,z=1,speed=0.6,frame_id='aruco_map')
rospy.sleep(15)

#set_position(x=0,y=1,z=1,frame_id='map')
#rospy.sleep(7)

#set_position(x=0,y=0,z=1,frame_id='map')
navigate(x=0,y=0,z=0.7,speed=0.4,frame_id='aruco_map')
rospy.sleep(11)

#navigate(x=0,y=0,z=1.5,frame_id='aruco_map')
#rospy.sleep(10)
print('Perform landing')
setAutoLandMode()
#set_position(x=0,y=0,z=-3,frame_id='body')
