# Information: https://clover.coex.tech/programming

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import threading
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Wrench
from gazebo_msgs.srv import ApplyBodyWrench, BodyRequest # For applying body wrench

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

# Could plot the stored data in SITL (not hardware) if desired:
import matplotlib
import matplotlib.pyplot as plt

# Want wrench service to apply force
apply_force = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)

# Define some variables to get the dryden windgust model force application ready
period2 = 9
period1 = 7
period = 7
body_name = 'clover::base_link'
reference_frame = 'clover::base_link'
rr = rospy.Rate(40)
start_stamp = rospy.get_rostime() # Start time of publishing
duration = rospy.Duration(secs = 30, nsecs = 0) # Not going to be synchronized, therefore writing a plugin is better (to synchronize with gazebo internally) however this is good enough

mass = Wrench() # Define class objecy
# Set the force and torque you want to apply:
mass.force.x = 0
mass.force.y = 0
mass.force.z = -0.2 # maybe something in 3-4 range -4.3 Hover thrust sim: -3.2N
mass.torque.x = 0
mass.torque.x = 0
mass.torque.x = 0
# Set the duration (<0 sets for rest of simulation)
duration_m = rospy.Duration(-1)


# For storing and plotting values
t = []
t1 = []
f = []
GE = []

# Dryden windgust model thread to be executed
def dryden_wind():
	while not stop_event.is_set():
		# Applying wrench disturbances (dryden windgust model)
		#wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0.8*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period2), \
		#y = 1.1*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period), z = 1.1*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period1) ), \
		#torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
		wrench =  geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3(x=0,y=0,z=0.8*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period1)),torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
		t.append((rospy.get_rostime() - start_stamp).to_sec())
		f.append(wrench.force.z)
			# Clear body wrench (it is cummulative so need to clear)
		clear_body_wrench(body_name)
		# Apply wrench force
			#apply_force(body_name = body_name, reference_frame = reference_frame, wrench = wrench, duration = duration)
		apply_force(body_name = body_name, wrench = wrench, duration = duration) # apply in inertial frame when left empty
		rr.sleep()
# Ground effect model thread to be executed
def GE_model():
	while not stop_GE.is_set():
		# Applying wrench disturbances (ground effect model)
		#wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0.8*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period2), \
		#y = 1.1*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period), z = 1.1*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec())/period1) ), \
		#torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
		telem = get_telemetry()
		wrench =  geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3(x=0,y=0,z=1.2*(1/(1-31*(0.12/(4*telem.z))**2))),torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
		t1.append((rospy.get_rostime() - start_stamp).to_sec())
		GE.append(wrench.force.z)
			# Clear body wrench (it is cummulative so need to clear)
		clear_body_wrench(body_name)
		# Apply wrench force
			#apply_force(body_name = body_name, reference_frame = reference_frame, wrench = wrench, duration = duration)
		apply_force(body_name = body_name, wrench = wrench, duration = duration) # apply in inertial frame when left empty
		rr.sleep()

# Create a stop event to wind gust model
stop_event = threading.Event()

stop_GE = threading.Event()

# Create and start the thread
thread = threading.Thread(target=dryden_wind)



print('Take off and hover 1 m above the ground')

navigate(x=0,y=0,z=1,frame_id='body',speed=0.2,auto_arm=True)

rospy.sleep(5)

thread.start()

rospy.sleep(19)



# tell the thread to stop and wait for it to finish
stop_event.set()
thread.join()
print('Dryden has stopped.')

navigate(x=0, y=0, z=0.24, speed = 0.1,frame_id='aruco_map')

# Wait for 5 seconds
threadGE = threading.Thread(target=GE_model)
threadGE.start()
rospy.sleep(20)
stop_GE.set()
threadGE.join()

print('GE_Model has stopped.')
navigate(x=0, y=0, z=0.55, speed=0.12,frame_id='aruco_map')
rospy.sleep(4)
rospy.sleep(9)
clear_body_wrench(body_name)
apply_force(body_name = 'clover::base_link',reference_frame='clover::base_link',wrench = mass,duration = duration_m)
rospy.sleep(11)

navigate(x=0, y=0, z=1.1, speed=0.12,frame_id='aruco_map')
rospy.sleep(17)
print('Perform landing')
land()

# Debug section, need matplotlib to plot the results
#plt.plot(t,velx)
#plt.plot(t,posx)
#plt.plot(t,afx)
#plt.plot(t,yawc)
#plt.plot(t,yaw_ratec)
#plt.show()
plt.figure(1)
plt.plot(t, f, drawstyle = "steps-post")
plt.show()
plt.figure(2)
plt.plot(t1, GE, drawstyle = "steps-post")
plt.show()
