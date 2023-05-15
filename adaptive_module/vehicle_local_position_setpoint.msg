# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

uint64 timestamp	# time since system start (microseconds)

float32 x		# in meters NED
float32 y		# in meters NED
float32 z		# in meters NED
float32 yaw		# in radians NED -PI..+PI
float32 yawspeed	# in radians/sec
float32 vx		# in meters/sec
float32 vy		# in meters/sec
float32 vz		# in meters/sec
float32[3] acceleration # in meters/sec^2
float32[3] jerk # in meters/sec^3
float32[3] thrust	# normalized thrust vector in NED

# Observer logging variables
float32 xhat0		# in meters NED
float32 xhat1		# in m/s NED
float32 xhat2		# in m/s^2 NED
float32 exp             # in m NED

float32 xhatep		        # in m NED
float32 xhatev		        # in m/s NED

float32 yhat0		# in meters NED
float32 yhat1		# in m/s NED
float32 yhat2		# in m/s^2 NED
float32 eyp             # in m NED

float32 yhatep		        # in m NED
float32 yhatev		        # in m/s NED

float32 zhat0		# in meters NED
float32 zhat1		# in m/s NED
float32 zhat2		# in m/s^2 NED
float32 ezp             # in m NED


float32 zhatep		        # in m NED
float32 zhatev		        # in m/s NED

float32 lx		        # adaptive param
float32 lambda1xa		# scaled adaptive param
float32 lambda2xa		# scaled adaptive param
float32 lambda3xa		# scaled adaptive param

float32 ly		        # adaptive param
float32 lambda1ya		# scaled adaptive param
float32 lambda2ya		# scaled adaptive param
float32 lambda3ya		# scaled adaptive param

float32 lz		        # adaptive param
float32 lambda1za		# scaled adaptive param
float32 lambda2za		# scaled adaptive param
float32 lambda3za		# scaled adaptive param


# TOPICS vehicle_local_position_setpoint trajectory_setpoint
