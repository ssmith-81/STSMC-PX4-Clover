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

wrench = Wrench()

wrench.force.x = 0
wrench.force.y = 50
wrench.force.z = 0
wrench.torque.x = 0
wrench.torque.x = 0
wrench.torque.x = 0



apply_force(body_name = 'clover::base_link',reference_frame='clover::base_link',wrench = wrench,duration = rospy.Duration(0.5))
