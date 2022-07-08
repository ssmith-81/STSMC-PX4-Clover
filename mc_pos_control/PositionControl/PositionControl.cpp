/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <ecl/geo/geo.h>
#include <fstream>
#include <iostream>

using namespace matrix;
using namespace std;

#define STSMC true    // Turn controller on
#define STSMC_HOSMO false    // Turn controller based on observer on

// This stuff doesnt work, module does not allow logging like this
//ofstream est;
//est.open("x_hat.txt");

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'

	_vel_int(2) += (hover_thrust_new - _hover_thrust) * (CONSTANTS_ONE_G / hover_thrust_new);
	setHoverThrust(hover_thrust_new);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	// Original update scripts
//-----------------------------------------------------------------------------------------------
	// x and y input setpoints always have to come in pairs
	const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
			   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)))
			   && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	//_positionControl();       //Turn these off, not using PID anymore
	//_velocityControl(dt);     // Not using this velocity module anymore
	SMC_control(dt);            // Use SMC controller
	

	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	return valid && _updateSuccessful();
//-------------------------------------------------------------------------------------------------
// Was trying to create own update functions but this was not needed:

// x and y input setpoints always have to come in pairs
	// const bool valid = (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)))
	// 		   && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
			   

	// SMC_control(dt);            // Use SMC controller

	// _yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	// _yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control

	// return valid && _updateSuccessful();
	
}

void PositionControl::SMC_control(const float dt)
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

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, U); // --> _acc_sp = _acc_sp + U  --> U becomes set point to be converted to a thrust and attitude

	// These logging commands dont work:
	//FILE *data = fopen("STSMC.txt", "a");
	//fprintf(data, "%f\t  %f\t  %f\t  \n", (double)Uxx, (double)Uyy, (double)Uzz);
	
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
	// this setpoint is then converted to a thrust and attitude setpoint.

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, U); // --> _acc_sp = _acc_sp + U  --> U becomes set point to be converted to a thrust and attitude

#endif

	_accelerationControl();

//-----------------------------------------------------------------------------------
// These are some thrust calculations and conversions taken out of the velocity control 
// function that are needed:

	//Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);
	//cout<<thrust_sp(2)<<endl;
	// Get allowed horizontal thrust after prioritizing vertical control
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}
}



//-------------------------------------------------------------------------------------

// Original PID position control module

// void PositionControl::_positionControl()
// {
// 	// P-position controller
// 	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
// 	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
// 	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
// 	// make sure there are no NAN elements for further reference while constraining
// 	ControlMath::setZeroIfNanVector3f(vel_sp_position);

// 	// Constrain horizontal velocity by prioritizing the velocity component along the
// 	// the desired position setpoint over the feed-forward term.
// 	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
// 	// Constrain velocity in z-direction.
// 	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down); 

// }

// void PositionControl::_velocityControl(const float dt)
// {
// 	// PID velocity control
// 	Vector3f vel_error = _vel_sp - _vel;
// 	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);


// 	// No control input from setpoints or corresponding states which are NAN
// 	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
// 	Uzz = acc_sp_velocity(2);
// 	_accelerationControl();

// 	//float x_dot2d = (_vel_sp(1) - pre_x_2d)/dt;

// 	// Integrator anti-windup in vertical direction
// 	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
// 	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
// 		vel_error(2) = 0.f;
// 	}

// 	// Saturate maximal vertical thrust
// 	_thr_sp(2) = math::max(_thr_sp(2), -_lim_thr_max);

// 	// Get allowed horizontal thrust after prioritizing vertical control
// 	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;
// 	const float thrust_z_squared = _thr_sp(2) * _thr_sp(2);
// 	const float thrust_max_xy_squared = thrust_max_squared - thrust_z_squared;
// 	float thrust_max_xy = 0;

// 	if (thrust_max_xy_squared > 0) {
// 		thrust_max_xy = sqrtf(thrust_max_xy_squared);
// 	}

// 	// Saturate thrust in horizontal direction
// 	const Vector2f thrust_sp_xy(_thr_sp);
// 	const float thrust_sp_xy_norm = thrust_sp_xy.norm();

// 	if (thrust_sp_xy_norm > thrust_max_xy) {
// 		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
// 	}

// 	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
// 	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
// 	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
// 	const float arw_gain = 2.f / _gain_vel_p(0);
// 	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

// 	// Make sure integral doesn't get NAN
// 	ControlMath::setZeroIfNanVector3f(vel_error);
// 	// Update integral part of velocity control
// 	_vel_int += vel_error.emult(_gain_vel_i) * dt;

// 	// limit thrust integral
// 	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));

// 	// Store values
// 	// FILE *data;

// 	// data = fopen("param.txt", "a");
// 	// fprintf(data, "%f\t  %f\t  %f \n", (double)acc_sp_velocity(2),(double)dt, (double)_thr_sp(2));
// 	// fclose(data);

// 	//cout << x_dot2d<<endl;
// 	//cout << acc_sp_velocity(1)<<_acc_sp(1)<<x_dot2d<<endl;
// 	//pre_x_2d = _vel_sp(1);

// }

void PositionControl::_accelerationControl()
{
	// Original acceleration (from velocity PID controller) to thrust
//----------------------------------------------------------------------------------------------------------
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	//cout<<collective_thrust<<" ";
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	//cout<<collective_thrust<<" "<<CONSTANTS_ONE_G<<" "<<_thr_sp(2)<<" "<<body_z(0)<<" "<<body_z(1)<<" "<<body_z(2)<<endl;
	_thr_sp = body_z * collective_thrust;
	cout<<_hover_thrust<<" "<<_lim_tilt<<" "<<_lim_thr_min<<endl;
	//cout<<_hover_thrust<<' '<<Uzz<<' '<<_acc_sp(2)<<' '<<_acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G)<<' '<<collective_thrust<<endl;
	//cout<<_hover_thrust<<' '<<Uzz<<' '<<_acc_sp(2)<<' '<<_acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G)<<' '<<collective_thrust<<endl;
//----------------------------------------------------------------------------------------------------------

	// Using thrust nonlinear decoupling equation for STSMC --> All of this is not needed. PX4 takes care of the 
	// acceleration to thrust and attitude conversions while maintaining normalized units therefore the non-linear
	// decoupling equations are not needed.

	float m = 1.5; //kg: mass of quadcopter --> redefine as a parameter later
	float g = 9.81; // m/s^2 gravitational constant

	thrust_z = m*sqrt((Uxx*Uxx)+(Uyy*Uyy)+(Uzz-g)*(Uzz-g));



	//thrust_z = math::min(-thrust_z, -_lim_thr_min);

	thrust_sp = Vector3f(-Uxx, -Uyy, g).normalized();

	//ControlMath::limitTilt(thrust_sp, Vector3f(0, 0, 1), _lim_tilt); // Limits tilt between thrust_sp and vertical
	// although the x and y of thrust_sp are set to zero so it will be vertical anyways with this model thrust

	//cout<<body_z(0)<<" "<<body_z(1)<<" "<<body_z(2)<<" "<<thrust_sp(0)<<" "<<thrust_sp(1)<<" "<<thrust_sp(2)<<endl;
	//cout<<_hover_thrust / CONSTANTS_ONE_G<<endl;
}

bool PositionControl::_updateSuccessful()
{
	bool valid = true;

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	// There has to be a valid output accleration and thrust setpoint otherwise there was no
	// setpoint-state pair for each axis that can get controlled
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));
	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
	//thrust_sp.copyTo(local_position_setpoint.thrust); // Was trying to use my own functions
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	// Original implementation that uses thrust converted from acceleration set point from PID velocity controller
	// This is the one that is used of course.
//-----------------------------------------------------------------------------------------
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
//-----------------------------------------------------------------------------------------
// Was trying to implement my own thrust to attitude conversion functions although these were not needed and did not work:
	// Takes calculated thrust setpoint, yaw setpoint which was obtained in above function, and attitude setpoint which
	// is a uORB topic to publish to i.e. attitude setpoint that the attitude controller will call in the attitude control module
	ControlMath::thrustToAttitudeSMC(thrust_sp,thrust_z,U, _yaw_sp, attitude_setpoint); 
	//attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp; // the uORB topic or struct attitude_setpoint.yaw_sp_move_rate is set with this
}
