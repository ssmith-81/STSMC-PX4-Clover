# Information: https://clover.coex.tech/programming

import rospy
import math
from clover import srv
from gazebo_msgs.srv import ApplyBodyWrench, BodyRequest
from std_srvs.srv import Trigger
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Wrench
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
import sys
  
import std_msgs
import time
import matplotlib.pyplot as plt
#from forces_torques.srv import *

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

wrench = Wrench()
navigate(x=0,y=0,z=1,frame_id='body',auto_arm=True)



def apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, start_time, duration):
	rospy.wait_for_service('/gazebo/apply_body_wrench')
	try:
		apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
		apply_body_wrench(body_name, reference_frame, reference_point, wrench, start_time, duration)
	except rospy.ServiceException as e:
		print ("Service arming call failed: %s")

def clear_body_wrench_client(body_name):
	rospy.wait_for_service('gazebo/clear_body_wrenches')
	try:
		clear_body_wrench = rospy.ServiceProxy('gazebo/clear_body_wrenches', BodyRequest)
		clear_body_wrench(body_name)
	except rospy.ServiceException as e:
		print ("Service arming call failed: %s")

if __name__ == "__main__":
	beginning = time.time()
	update_rate = 0.01
	period = 2
	period1 = 1
	body_name = 'clover::base_link'
	reference_frame = 'clover::base_link'
	start_time = rospy.Time(secs = 0, nsecs = 0)
	start_stamp = rospy.get_rostime() # Start time of publishing
	duration = rospy.Duration(secs = update_rate, nsecs = 0)
	reference_point = geometry_msgs.msg.Point(x = 0, y = 0, z = 0)
	t = []
	f = []
	while time.time()-beginning <= 10:
		wrench = geometry_msgs.msg.Wrench(force = geometry_msgs.msg.Vector3( x = 0, \
			y = 1.3*math.sin(2*math.pi*((rospy.get_rostime() - start_stamp).to_sec()-1)/period)+1.1*math.sin(math.pi*((rospy.get_rostime() - start_stamp).to_sec()+2)/period1), z = 0), \
			torque = geometry_msgs.msg.Vector3(x = 0, y = 0, z = 0))
		t.append(time.time() - beginning)
		f.append(wrench.force.y)
		clear_body_wrench_client(body_name)
		apply_body_wrench_client(body_name, reference_frame, reference_point, wrench, \
			start_time, duration)
		time.sleep(update_rate - ((time.time()-beginning)%update_rate))
land()
plt.plot(t, f, drawstyle = "steps-post")
plt.show()
