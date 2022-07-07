/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file PositionControl.cpp
 *
 * This file implements a P-position-control cascaded with a
 * PID-velocity-controller.
 *
 * Inputs: vehicle states (pos, vel, q)
 *         desired setpoints (pos, vel, thrust, yaw)
 * Outputs: thrust and yaw setpoint
 */

#include "PositionControl.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include "uORB/topics/parameter_update.h"
#include "Utility/ControlMath.hpp"

using namespace matrix;
using namespace ControlMath;

#define STSMC true // turn controller on
// This is for the controller-observer pair:
#define STSMC_HOSMO true// Turn on controller-observer pair

PositionControl::PositionControl()
{
	_Pz_h   = param_find("MPC_Z_P");
	_Pvz_h  = param_find("MPC_Z_VEL_P");
	_Ivz_h  = param_find("MPC_Z_VEL_I");
	_Dvz_h  = param_find("MPC_Z_VEL_D");
	_Pxy_h  = param_find("MPC_XY_P");
	_Pvxy_h = param_find("MPC_XY_VEL_P");
	_Ivxy_h = param_find("MPC_XY_VEL_I");
	_Dvxy_h = param_find("MPC_XY_VEL_D");
	_VelMaxXY_h = param_find("MPC_XY_VEL_MAX");
	_VelMaxZdown_h = param_find("MPC_Z_VEL_MAX_DN");
	_VelMaxZup_h = param_find("MPC_Z_VEL_MAX_UP");
	_ThrHover_h = param_find("MPC_THR_HOVER");
	_ThrMax_h = param_find("MPC_THR_MAX");
	_ThrMin_h = param_find("MPC_THR_MIN");

	/* Set parameter the very first time. */
	_setParams();
};

void PositionControl::updateState(const vehicle_local_position_s &state, const Vector3f &vel_dot)
{
	_pos = Vector3f(&state.x);
	_vel = Vector3f(&state.vx);
	_yaw = state.yaw;
	_vel_dot = vel_dot;
}

void PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(&setpoint.x);
	_vel_sp = Vector3f(&setpoint.vx);
	_acc_sp = Vector3f(&setpoint.acc_x);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	_interfaceMapping();

	/* If full manual is required (thrust already generated), don't run position/velocity
	 * controller and just return thrust.
	 */
	_skipController = false;

	if (PX4_ISFINITE(setpoint.thrust[0]) && PX4_ISFINITE(setpoint.thrust[1]) && PX4_ISFINITE(setpoint.thrust[2])) {
		_skipController = true;
	}
}

void PositionControl::generateThrustYawSetpoint(const float &dt)
{
	_updateParams();

	/* Only run position/velocity controller
	 * if thrust needs to be generated
	 */
	if (!_skipController) {
		//_positionController();
		//_velocityController(dt);
		SMC_control(dt);       // Use SMC controller
	}
}

void PositionControl::_interfaceMapping()
{
	/* Respects FlightTask interface, where
	 * NAN-setpoints are of no interest and
	 * do not require control.
	 */

	/* Loop through x,y and z components of all setpoints. */
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {

			/* Position control is required */

			if (!PX4_ISFINITE(_vel_sp(i))) {
				/* Velocity is not used as feedforward term. */
				_vel_sp(i) = 0.0f;
			}

			/* thrust setpoint is not supported
			 * in position control
			 */
			_thr_sp(i) = 0.0f;

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			/* Velocity controller is active without
			 * position control.
			 */
			_pos_sp(i) = _pos(i);
			_thr_sp(i) = 0.0f;

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			/* Thrust setpoint was generated from
			 * stick directly.
			 */
			_pos_sp(i) = _pos(i);
			_vel_sp(i) = _vel(i);
			_thr_int(i) = 0.0f;
			_vel_dot(i) = 0.0f;

		} else {
			PX4_WARN("TODO: add safety");
		}
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		_yaw_sp = _yaw;
	}
}

void PositionControl::SMC_control(const float &dt)
{
	// Can use the cout statement below to check the values of the velocity setpoints.
	 //cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<endl;

	// Use this when only a position setpoint is given (numerically calculate velocity setpoint):
	vel_spi(0) = (_pos_sp(0)-pos_xi)/dt;
	vel_spi(1) = (_pos_sp(1)-pos_yi)/dt;
	vel_spi(2) = (_pos_sp(2)-pos_zi)/dt;

	// Older versions of the firmware have the velocity setpoint pre-set to zero (In simulation anyway) so the first && conditions take care of that case
	// when they are all zero and unused; this uses the numerically calculated ones instead!
	// Newer versions of the firmware (in simulation) have the velocity setpoints as NaN when not published too, therefore the || conditions take care of that and
	// use the numerically calculated ones instead.
 if ((fabs(_vel_sp(0)-0.0f)<1e-9f && fabs(_vel_sp(1)-0.0f)<1e-9f && fabs(_vel_sp(2)-0.0f)<1e-9f) || !PX4_ISFINITE(_vel_sp(0)) || !PX4_ISFINITE(_vel_sp(1)) || !PX4_ISFINITE(_vel_sp(2))) // comparing floats 
 {
	// Use numerically calculated velocities
	 _vel_sp = vel_spi;
 }
 else{
	 // Use this when a velocity setpoint is given (such as commands from QGround Control
	// or when the landing function is used, it provides velocity setpoints so the numerical ones
	// are not needed).
	 _vel_sp = _vel_sp;
 }
 
	// Calculate acceleration of desired setpint for sliding mode controller:
	
	float x_ddot2d = (_vel_sp(0) - pre_xdot_2d) / dt; // define pre values in hpp file as zero
	float y_ddot2d = (_vel_sp(1) - pre_ydot_2d) / dt;
	float z_ddot2d = (_vel_sp(2) - pre_zdot_2d) / dt;
	
	// These are not needed anymore, were used previously with the dual controller setup:
	// float x_ddot2di = (vel_spi(0) - pre_xdot_2di) / dt; // define pre values in hpp file as zero
	// float y_ddot2di = (vel_spi(1) - pre_ydot_2di) / dt;
	// float z_ddot2di = (vel_spi(2) - pre_zdot_2di) / dt;

	//cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<endl;

#if STSMC

	 //************* Start of controller:

 // Position controller gains

 // float k1x = 3, k2x = 4;
 // float k1y = 3, k2y = 3;
 // float k1z = 4, k2z = 3;
 // Tuning notes:
 // Increasing k1 makes it more responsive in the plane but also very jittery once it is around like 2

 //Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving bck and forth
 float k1x = 1, k2x = 0.3; // 0.25
 float k1y = 1, k2y = 0.3;
 float k1z = 1, k2z = 1.5;

 //Error gains
 //float c1x = 3, c1y = 3, c1z = 3;

 float c1x = 0.75, c1y = 0.75, c1z = 1;

 // Sliding manifold
 // Error definitions:
 Vector3f e_1 = _pos - _pos_sp; //x(0)-x_1d;
 Vector3f e_2 = _vel - _vel_sp; //x(1) - x_2d;

 float sx = c1x * e_1(0) + (e_2(0));
 float sy = c1y * e_1(1) + (e_2(1));
 float sz = c1z * e_1(2) + (e_2(2));

 // Use saturation function instead of sign function
 float sat_sx = ControlMath::sat(sx);
 float sat_sy = ControlMath::sat(sy);
 float sat_sz = ControlMath::sat(sz);

 sat_sxin += sat_sx * dt;
 sat_syin += sat_sy * dt;
 sat_szin += sat_sz * dt;

 //Switching control terms
 sign_sx_int += k2x * sign(sx) * dt;
 sign_sy_int += k2y * sign(sy) * dt;
 sign_sz_int += k2z * sign(sz) * dt;

 // Define virtual controllers
 //float U_x = 0, U_y = 0, U_z = 0;

 // Uxx = 0.0f;
 // Uyy = 0.0f;
 // Uzz = 0.0f;

 Uxx = -c1x * (e_2(0)) + x_ddot2d - k1x * sqrt(abs(sx)) * sign(sx) - sign_sx_int;
 Uyy = -c1y * (e_2(1)) + y_ddot2d - k1y * sqrt(abs(sy)) * sign(sy) - sign_sy_int;
 Uzz = -c1z * (e_2(2)) + z_ddot2d - k1z * sqrt(abs(sz)) * sign(sz) - sign_sz_int;

 if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
 { // Use PX4 module
     Uxx = 0.0f;
     Uyy = 0.0f;
     Uzz = 0.0f;
 }

 //cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<" "<<!PX4_ISFINITE(_vel_sp(0))<<endl;

 // Save references to previous references:
 pre_xdot_2d = _vel_sp(0);
 pre_ydot_2d = _vel_sp(1);
 pre_zdot_2d = _vel_sp(2);
 
 // Save references to previous references for numerical calculations:
 pos_xi = _pos_sp(0);
 pos_yi = _pos_sp(1);
 pos_zi = _pos_sp(2);

 // Not needed anymore:
 //  pre_xdot_2di = vel_spi(0);
 //  pre_ydot_2di = vel_spi(1);
 //  pre_zdot_2di = vel_spi(2);
 //cout << _pos(2)<<_pos_sp(2)<<_vel(2)<<_vel_sp(2)<<endl;

 // Gather virtual control inputs into a vector
 U = Vector3f(Uxx, Uyy, Uzz);
              
 // Add these virtual control inputs to the acceleration setpoint (this is what the velocity PID controller produces)
 // this setpoint is then converted to a thrust and attitude setpoint. This is what the non-linear
 // decoupling equations are supposed to do, but PX4 takes advantage of quadcopter dynamics being differentially flat and use other methods.
 
#endif

//****************************************************************************************

	#if STSMC_HOSMO

		
		// Position controller gains
		
		// float k1x = 3, k2x = 4;
		// float k1y = 3, k2y = 3;
		// float k1z = 4, k2z = 3;
		// Tuning notes:
		// Increasing k1 makes it more responsive in the plane but also very jittery once it is around like 2

		//Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving bck and forth
		float k1x = 1, k2x = 0.3; // 0.25
		float k1y = 1, k2y = 0.3;
		float k1z = 1, k2z = 1.5;

		// Observer gains
		float lambda1x = 5, lambda2x = 20, lambda3x = 0.5;
		float lambda1y = 5, lambda2y = 20, lambda3y = 7;
		float lambda1z = 5, lambda2z = 20, lambda3z = 7;

		//Error gains
		//float c1x = 0.75, c1y = 0.75, c1z = 1;

		float c1x = 0.75, c1y = 0.75, c1z = 1;

		// Observer based control


		// Set the estimation (observer) initial conditions. Overwrties any elements of a Vector3f which are
		// NaN with zero
		ControlMath::setZeroIfNanVector3f(x_hat); /**<x-dynamic estimated states */
		ControlMath::setZeroIfNanVector3f(y_hat); /**<y-dynamic estimated states */
		ControlMath::setZeroIfNanVector3f(z_hat); /**<z-dynamic estimated states */

		// Define estimation errors
		float x_tilde = _pos(0) - x_hat(0);
		float y_tilde = _pos(1) - y_hat(0);
		float z_tilde = _pos(2) - z_hat(0);

		// Define saturation functions (smoothing functions) to replace sign(s) in virtual controllers
		// float epsilon = 0.000001;
		// float sat_z = ControlMath::sat(z_tilde/epsilon);

		// Compute estimates with observer
		if (!PX4_ISFINITE(Ux) || !PX4_ISFINITE(Uy) || !PX4_ISFINITE(Uz))
		{ // Use PX4 module
			Ux = 0.0f;
			Uy = 0.0f;
			Uz = 0.0f;
		}

		u_x = Ux;
		u_y = Uy;
		u_z = Uz;

		// x-dynamics observer  // x_hat += x_hat... --> integral
		x_hat(0) = x_hat(0) + (x_hat(1) * dt) + lambda1x * cbrt(fabsf(x_tilde) * fabsf(x_tilde)) * sign(x_tilde) * dt;
		x_hat(1) = x_hat(1) + (x_hat(2) * dt) + lambda2x * cbrt(fabsf(x_tilde)) * sign(x_tilde) * dt + (u_x)*dt;
		x_hat(2) = x_hat(2) + lambda3x * sign(x_tilde) * dt;
		//x_hat(2) = x_hat(2) + lambda3x*sat_x*dt;  // Disturbance estimation with smoothing function
		// y-dynamics observer
		y_hat(0) = y_hat(0) + (y_hat(1) * dt) + lambda1y * cbrt(fabsf(y_tilde) * fabsf(y_tilde)) * sign(y_tilde) * dt;
		y_hat(1) = y_hat(1) + (y_hat(2) * dt) + lambda2y * cbrt(fabsf(y_tilde)) * sign(y_tilde) * dt + (u_y)*dt;
		y_hat(2) = y_hat(2) + lambda3y * sign(y_tilde) * dt;
		//y_hat(2) = y_hat(2) + lambda3y*sat_y*dt;  // Disturbance estimation with smoothing function
		// z-dynamics observer**
		z_hat(0) = z_hat(0) + (z_hat(1) * dt) + lambda1z * cbrt(fabsf(z_tilde) * fabsf(z_tilde)) * sign(z_tilde) * dt;
		z_hat(1) = z_hat(1) + (z_hat(2) * dt) + lambda2z * cbrt(fabsf(z_tilde)) * sign(z_tilde) * dt + (u_z)*dt;
		z_hat(2) = z_hat(2) + lambda3z * sign(z_tilde) * dt;
		//z_hat(2) = z_hat(2) + lambda3z*sat_z*dt;  // Disturbance estimation with smoothing function

		// Sliding manifold
		// Error definitions:
		float error_x = _pos(0) - _pos_sp(0);
		float error_y = _pos(1) - _pos_sp(1);
		float error_z = _pos(2) - _pos_sp(2); //z(0)-z_1d;

		// For non observer based STSMC altitude controller
		//Vector3f e_2 = _vel - _vel_sp; //x(1) - x_2d;

		float sx = c1x * error_x + (x_hat(1) - _vel_sp(0));
		float sy = c1y * error_y + (y_hat(1) - _vel_sp(1));
		// Use observer for z-dynamics
		float sz = c1z*error_z + (z_hat(1)-_vel_sp(2));
		// Use super twisting sliding mode controller only based on sensor feedback for z-dynamic (No Observer)
		//float sz = c1z * error_z + (_vel(2) - _vel_sp(2))

		
		//Switching control terms
		sign_sx_int += k2x * sign(sx) * dt;
		sign_sy_int += k2y * sign(sy) * dt;
		sign_sz_int += k2z * sign(sz) * dt;

		// Saturation instead of sign for switching control portion. (can use if you would like)
		//flooat sat_sx = ControlMath::sat(sx);
		//flat sat_sy = ControlMath::sat(sy);
		//float sat_sz = ControlMath::sat(sz);
		//Integral terms for controllers
		//sat_sx_int += k2x*sat_sx*dt
		//sat_sy_int += k2y*sat_sy*dt
		//sat_sz_int += k2z*sat_sz*dt

		// Define virtual controllers
		// float Ux = 0;
		// float Uy = 0;
		// float Uz = 0;

		Ux = -c1x * x_hat(1) + c1x * _vel_sp(0) - x_hat(2) - lambda2x * cbrt(fabsf(x_tilde)) * sign(x_tilde) * dt + x_ddot2d - k1x * sqrt(fabs(sx)) * sign(sx) - sign_sx_int;
		Uy = -c1y * y_hat(1) + c1y * _vel_sp(1) - y_hat(2) - lambda2y * cbrt(fabsf(y_tilde)) * sign(y_tilde) * dt + y_ddot2d - k1y * sqrt(fabs(sy)) * sign(sy) - sign_sy_int;

		// Altitude controller:
		// Based on observer:
		Uz = -c1z*z_hat(1) + c1z*_vel_sp(2) - z_hat(2) - lambda2z*cbrt(fabsf(z_tilde))*sign(z_tilde)*dt + z_ddot2d - k1z*sqrt(fabs(sz))*sign(sz) - sign_sz_int;
		// STSMC with no Observer
		//Uz = -c1z * (e_2(2)) + z_ddot2d - k1z * sqrt(abs(sz)) * sign(sz) - sign_sz_int;

		// Save references to previous references:
		pre_xdot_2d = _vel_sp(0);
		pre_ydot_2d = _vel_sp(1);
		pre_zdot_2d = _vel_sp(2);

		// Save references to previous references for numerical calculations:
		pos_xi = _pos_sp(0);
		pos_yi = _pos_sp(1);
		pos_zi = _pos_sp(2);

		// Not needed anymore:
		//  pre_xdot_2di = vel_spi(0);
		//  pre_ydot_2di = vel_spi(1);
		//  pre_zdot_2di = vel_spi(2);
		//cout << _pos(2)<<_pos_sp(2)<<_vel(2)<<_vel_sp(2)<<endl;

		if (!PX4_ISFINITE(Ux) || !PX4_ISFINITE(Uy) || !PX4_ISFINITE(Uz))
		{ // Use PX4 module
			Ux = 0.0f;
			Uy = 0.0f;
			Uz = 0.0f;
		}

		//cout<<_pos(0)-x_hat(0)<<" "<<_vel(0)-x_hat(1)<<" "<<x_hat(2)<<" "<<Ux<<" "<<Uy<<endl;

		// Gather virtual control inputs into a vector
		U = Vector3f(Ux, Uy, Uz);

		// Add these virtual control inputs to the acceleration setpoint (this is what the velocity PID controller produces)
 		// this setpoint is then converted to a thrust and attitude setpoint. This is what the non-linear
 		// decoupling equations are supposed to do, but PX4 takes advantage of quadcopter dynamics being differentially flat and use other methods.


		
		#endif
	//*****************************************************************************************

	/* Get maximum tilt */
	float tilt_max = M_PI_2_F;

	if (PX4_ISFINITE(_constraints.tilt_max) && _constraints.tilt_max <= tilt_max) {
		tilt_max = _constraints.tilt_max;
	}

//Acceleration control function from newer PX4 firmware
//Assume standard acceleration due to gravity in vertical direction for attitude generation
	//Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	float CONSTANTS_ONE_G = 9.81; // Gravitational constant
	Vector3f body_z = Vector3f(-U(0), -U(1), CONSTANTS_ONE_G).normalized();
	
	float _lim_tilt = tilt_max;  // max tilt in radians
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	float _hover_thrust = _ThrHover;
	//Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = U(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	//Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	//collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;


//-----------------------------------------------------------------------------------
// These are some thrust calculations and conversions taken out of the velocity control
// function that are needed:


/* The Thrust limits are negated and swapped due to NED-frame */
	float uMax = -_ThrustLimit.min;
	float uMin = -_ThrustLimit.max;

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = math::min(uMax, -10e-4f);

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(_thr_sp(2), uMin, uMax);

	
		if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) {

		/* Thrust setpoints in NE-direction is already provided. Only
		 * scaling is required.
		 */

		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(tilt_max);
		_thr_sp(0) *= thr_xy_max;
		_thr_sp(1) *= thr_xy_max;

	}
	
	
}


// Original PID position control module

//--------------------------------------------------------------------------------------------------------------------
// void PositionControl::_positionController()
// {
// 	/* Generate desired velocity setpoint */

// 	/* P-controller */
// 	_vel_sp = (_pos_sp - _pos).emult(Pp) + _vel_sp;

// 	/* Make sure velocity setpoint is constrained in all directions (xyz). */
// 	float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) + _vel_sp(1) * _vel_sp(1));

// 	if (vel_norm_xy > _VelMaxXY) {
// 		_vel_sp(0) = _vel_sp(0) * _VelMaxXY / vel_norm_xy;
// 		_vel_sp(1) = _vel_sp(1) * _VelMaxXY / vel_norm_xy;
// 	}

// 	/* Saturate velocity in D-direction */
// 	_vel_sp(2) = math::constrain(_vel_sp(2), -_VelMaxZ.up, _VelMaxZ.down);
// }

// void PositionControl::_velocityController(const float &dt)
// {
// 	/* Generate desired thrust setpoint */
// 	/* PID
// 	 * u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
// 	 * Umin <= u_des <= Umax
// 	 *
// 	 * Anti-Windup:
// 	 * u_des = _thr_sp; r = _vel_sp; y = _vel
// 	 * u_des >= Umax and r - y >= 0 => Saturation = true
// 	 * u_des >= Umax and r - y <= 0 => Saturation = false
// 	 * u_des <= Umin and r - y <= 0 => Saturation = true
// 	 * u_des <= Umin and r - y >= 0 => Saturation = false
// 	 *
// 	 *	Notes:
// 	 * - PID implementation is in NED-frame
// 	 * - control output in D-direction has priority over NE-direction
// 	 * - the equilibrium point for the PID is at hover-thrust
// 	 * - the maximum tilt cannot exceed 90 degrees. This means that it is
// 	 * 	 not possible to have a desired thrust direction pointing in the positive
// 	 * 	 D-direction (= downward)
// 	 * - the desired thrust in D-direction is limited by the thrust limits
// 	 * - the desired thrust in NE-direction is limited by the thrust excess after
// 	 * 	 consideration of the desired thrust in D-direction. In addition, the thrust in
// 	 * 	 NE-direction is also limited by the maximum tilt.
// 	 */

// 	/* Get maximum tilt */
// 	float tilt_max = M_PI_2_F;

// 	if (PX4_ISFINITE(_constraints.tilt_max) && _constraints.tilt_max <= tilt_max) {
// 		tilt_max = _constraints.tilt_max;
// 	}

// 	Vector3f vel_err = _vel_sp - _vel;

// 	/*
// 	 * TODO: add offboard acceleration mode
// 	 * */

// 	/* Consider thrust in D-direction */
// 	float thrust_desired_D = Pv(2) * vel_err(2) + Dv(2) * _vel_dot(2) + _thr_int(2) - _ThrHover;

// 	/* The Thrust limits are negated and swapped due to NED-frame */
// 	float uMax = -_ThrustLimit.min;
// 	float uMin = -_ThrustLimit.max;

// 	/* Apply Anti-Windup in D-direction */
// 	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
// 			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

// 	if (!stop_integral_D) {
// 		_thr_int(2) += vel_err(2) * Iv(2) * dt;

// 	}

// 	/* Saturate thrust setpoint in D-direction */
// 	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

// 	if (fabsf(_thr_sp(0)) + fabsf(_thr_sp(1))  > FLT_EPSILON) {

// 		/* Thrust setpoints in NE-direction is already provided. Only
// 		 * scaling is required.
// 		 */

// 		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(tilt_max);
// 		_thr_sp(0) *= thr_xy_max;
// 		_thr_sp(1) *= thr_xy_max;

// 	} else {

// 		/* PID for NE-direction */
// 		Vector2f thrust_desired_NE;
// 		thrust_desired_NE(0) = Pv(0) * vel_err(0) + Dv(0) * _vel_dot(0) + _thr_int(0);
// 		thrust_desired_NE(1) = Pv(1) * vel_err(1) + Dv(1) * _vel_dot(1) + _thr_int(1);

// 		/* Get maximum allowed thrust in NE based on tilt and excess thrust */
// 		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(tilt_max);
// 		float thrust_max_NE = sqrtf(_ThrustLimit.max * _ThrustLimit.max - _thr_sp(2) * _thr_sp(2));
// 		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

// 		/* Get the direction of (r-y) in NE-direction */
// 		float direction_NE = Vector2f(&vel_err(0)) * Vector2f(&_vel_sp(0));

// 		/* Apply Anti-Windup in NE-direction */
// 		bool stop_integral_NE = (thrust_desired_NE * thrust_desired_NE >= thrust_max_NE * thrust_max_NE &&
// 					 direction_NE >= 0.0f);

// 		if (!stop_integral_NE) {
// 			_thr_int(0) += vel_err(0) * Iv(0) * dt;
// 			_thr_int(1) += vel_err(1) * Iv(1) * dt;
// 		}

// 		/* Saturate thrust in NE-direction */
// 		_thr_sp(0) = thrust_desired_NE(0);
// 		_thr_sp(1) = thrust_desired_NE(1);

// 		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
// 			float mag = thrust_desired_NE.length();
// 			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
// 			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;

// 		}
// 	}

// }

void PositionControl::updateConstraints(const Controller::Constraints &constraints)
{
	_constraints = constraints;
}

void PositionControl::_updateParams()
{
	bool updated;
	parameter_update_s param_update;
	orb_check(_parameter_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _parameter_sub, &param_update);
		_setParams();
	}
}

void PositionControl::_setParams()
{
	param_get(_Pxy_h, &Pp(0));
	param_get(_Pxy_h, &Pp(1));
	param_get(_Pz_h, &Pp(2));

	param_get(_Pvxy_h, &Pv(0));
	param_get(_Pvxy_h, &Pv(1));
	param_get(_Pvz_h, &Pv(2));

	param_get(_Ivxy_h, &Iv(0));
	param_get(_Ivxy_h, &Iv(1));
	param_get(_Ivz_h, &Iv(2));

	param_get(_Dvxy_h, &Dv(0));
	param_get(_Dvxy_h, &Dv(1));
	param_get(_Dvz_h, &Dv(2));

	param_get(_VelMaxXY_h, &_VelMaxXY);
	param_get(_VelMaxZup_h, &_VelMaxZ.up);
	param_get(_VelMaxZdown_h, &_VelMaxZ.down);

	param_get(_ThrHover_h, &_ThrHover);
	param_get(_ThrMax_h, &_ThrustLimit.max);
	param_get(_ThrMin_h, &_ThrustLimit.min);
}
