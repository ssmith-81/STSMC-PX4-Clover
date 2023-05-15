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
#include <geo/geo.h>
#include <iostream>
#include <math.h> // for hyperbolic tan function --> use on hardware

using namespace matrix;
using namespace std;
using namespace ControlMath;


#define STSMC true    // Turn controller on
#define STSMC_HOSMO false    // Turn controller based on observer on
#define test false


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

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
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
	if (hover_thrust_new > FLT_EPSILON) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		// Uzz += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		 hyp_sz_int -= (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		// e1_int -= (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		// z_hat(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * _hover_thrust / hover_thrust_new + CONSTANTS_ONE_G - _acc_sp(2);
		
		setHoverThrust(hover_thrust_new);
	}
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setAttitude(const AttitudeStates &attitude)
{
	yaw_s = attitude.yaw;

	pitch_s = attitude.pitch;

	roll_s = attitude.roll;
}

void PositionControl::setInputSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// These are feedforward updates
	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration); // --> feedforward component of desired acceleration is used in STSMC so this works out, no changes needed
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt)
{
	bool valid = _inputValid();

//******** Use SMC controller **********************

	// if (valid) {

	// 	// SMC_control(dt);            // Use SMC controller

	// if (!PX4_ISFINITE(_vel_sp(0)) || !PX4_ISFINITE(_vel_sp(1)) || !PX4_ISFINITE(_vel_sp(2))){

	// SMC_control(dt);            // Use SMC controller
	// }

	// else{
	// 	_positionControl();			// Turn this off, not using PID anymore
	// 	_velocityControl(dt);		// Not using this velocity module anymore
	// }	

	// 	_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
	// 	_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	// }

	//*********************Use PID controller *******************************

	if (valid) {

	
		_positionControl();			
		_velocityControl(dt);		
	

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}
	//************************************************************************

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	valid = valid && PX4_ISFINITE(_acc_sp(0)) && PX4_ISFINITE(_acc_sp(1)) && PX4_ISFINITE(_acc_sp(2));
	valid = valid && PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1)) && PX4_ISFINITE(_thr_sp(2));

	return valid;
}



void PositionControl::SMC_control(const float dt)
{
	// Can use the cout statement below to check the values of the velocity setpoints.
	 //cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<endl;

	// Use this when only a position setpoint is given (numerically calculate velocity setpoint):
	vel_spi(0) = (_pos_sp(0)-pos_xi)/dt;
	vel_spi(1) = (_pos_sp(1)-pos_yi)/dt;
	vel_spi(2) = (_pos_sp(2)-pos_zi)/dt;

	

	    // Should not use velocity setpoints provided from another source,
		// the STSMC is designed for position control and not velocity control and using random velocities 
		// will just mess the control up. It expects velocities calculated from position parameters and if random 
		// velocities are taken in then the controller is not designed to handle that.
		//-----------------------------------------------------------------------------------------------

		

	//cout<<_vel_sp(2)<<" "<<vel_spi(2)<<" "<<_pos_sp(2)<<" "<<Uxx<<" "<<!PX4_ISFINITE(_vel_sp(0))<<endl;
	
		// Calculate acceleration of desired setpint for sliding mode controller:
		
		// float x_ddot2d = (_vel_sp(0) - pre_xdot_2d) / dt; // define pre values in hpp file as zero
		// float y_ddot2d = (_vel_sp(1) - pre_ydot_2d) / dt;
		// float z_ddot2d = (_vel_sp(2) - pre_zdot_2d) / dt;
		
		// These are not needed anymore, were used previously with the dual controller setup:
		float x_ddot2di = (vel_spi(0) - pre_xdot_2di) / dt; // define pre values in hpp file as zero
		float y_ddot2di = (vel_spi(1) - pre_ydot_2di) / dt;
		float z_ddot2di = (vel_spi(2) - pre_zdot_2di) / dt;

		//cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<endl;

	#if STSMC

		//************* Start of controller:

	// Position controller gains

	// float k1x = 3, k2x = 4;
	// float k1y = 3, k2y = 3;
	// float k1z = 4, k2z = 3;
	// Tuning notes:
	// Increasing k1 makes it more responsive in the plane but also very jittery once it is around like 2

	//Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving back and forth
	float k1x = 1, k2x = 0.1; // 0.3
	float k1y = 1, k2y = 0.1;
	float k1z = 1, k2z = 1.5;

	//Error gains
	//float c1x = 3, c1y = 3, c1z = 3;

	float c1x = 0.75, c1y = 0.75, c1z = 1;

	// Sliding manifold
	// Error definitions:
	Vector3f e_1 = _pos - _pos_sp; //x(0)-x_1d;
	//Vector3f e_2 = _vel - vel_spi; //x(1) - x_2d;
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

	Uxx = -c1x * (e_2(0)) + x_ddot2di - k1x * sqrt(fabs(sx)) * sign(sx) - sign_sx_int;
	Uyy = -c1y * (e_2(1)) + y_ddot2di - k1y * sqrt(fabs(sy)) * sign(sy) - sign_sy_int;
	Uzz = -c1z * (e_2(2)) + z_ddot2di - k1z * sqrt(fabs(sz)) * sign(sz) - sign_sz_int;

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
	 pre_xdot_2di = vel_spi(0);
	 pre_ydot_2di = vel_spi(1);
	 pre_zdot_2di = vel_spi(2);

	//cout << _pos(2)<<""<<_pos_sp(2)<<""<<_vel(2)<<""<<vel_spi(2)<<endl;
	//cout<<"SMC"<<" "<<_vel_sp(0)<<endl;
	cout<<_vel(0)<<" "<<_vel_sp(0)<<" "<<vel_spi(0)<<endl;

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

		// // Observer gains
		// float lambda1x = 5, lambda2x = 20, lambda3x = 0.5;
		// float lambda1y = 5, lambda2y = 20, lambda3y = 7;
		// float lambda1z = 5, lambda2z = 20, lambda3z = 7;

		// Observer gains
		float lambda1x = 5, lambda2x = 10, lambda3x = 0.5;
		float lambda1y = 5, lambda2y = 10, lambda3y = 0.5;
		float lambda1z = 5, lambda2z = 10, lambda3z = 0.5;

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
		x_hat(0) = x_hat(0)  + (x_hat(1) * dt) + lambda1x * cbrt(fabsf(x_tilde) * fabsf(x_tilde)) * sign(x_tilde) * dt;
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

		float sx = c1x * error_x + (x_hat(1) - vel_spi(0));
		float sy = c1y * error_y + (y_hat(1) - vel_spi(1));
		// Use observer for z-dynamics
		float sz = c1z*error_z + (z_hat(1)-vel_spi(2));
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
		Ux = 0;
		Uy = 0;
		Uz = 0;

		Ux = -c1x * x_hat(1) + c1x * vel_spi(0) - x_hat(2) - lambda2x * cbrt(fabsf(x_tilde)) * sign(x_tilde) + x_ddot2di - k1x * sqrt(fabs(sx)) * sign(sx) - sign_sx_int;
		Uy = -c1y * y_hat(1) + c1y * vel_spi(1) - y_hat(2) - lambda2y * cbrt(fabsf(y_tilde)) * sign(y_tilde) + y_ddot2di - k1y * sqrt(fabs(sy)) * sign(sy) - sign_sy_int;

		// Altitude controller:
		// Based on observer:
		Uz = -c1z*z_hat(1) + c1z*vel_spi(2) - z_hat(2) - lambda2z*cbrt(fabsf(z_tilde))*sign(z_tilde) + z_ddot2di - k1z*sqrt(fabs(sz))*sign(sz) - sign_sz_int;
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
		 pre_xdot_2di = vel_spi(0);
		 pre_ydot_2di = vel_spi(1);
		 pre_zdot_2di = vel_spi(2);
		//cout << _pos(2)<<_pos_sp(2)<<_vel(2)<<_vel_sp(2)<<endl;

		if (!PX4_ISFINITE(Ux) || !PX4_ISFINITE(Uy) || !PX4_ISFINITE(Uz))
		{ // Use PX4 module
			Ux = 0.0f;
			Uy = 0.0f;
			Uz = 0.0f;
		}

		cout<<_pos(0)-x_hat(0)<<" "<<_pos(1)-y_hat(0)<<" "<<_pos(2)-z_hat(0)<<endl
		// cout<<_pos(0)-x_hat(0)<<" "<<_vel(0)-x_hat(1)<<" "<<x_hat(2)<<" "<<Ux<<" "<<Uy<<endl;
		//cout<<z_hat(2)<<endl;
		// Gather virtual control inputs into a vector
		U = Vector3f(Ux, Uy, Uz);


	// Add these virtual control inputs to the acceleration setpoint (this is what the velocity PID controller produces)
	// this setpoint is then converted to a thrust and attitude setpoint.

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, U); // --> _acc_sp = _acc_sp + U  --> U becomes set point to be converted to a thrust and attitude

#endif

#if test

		//cout<<_param_mpc_z_p<<endl;
	//Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving bck and forth
		// float k1x = 1, k2x = 0.3; // 0.25
		// float k1y = 1, k2y = 0.3;
		// float k1z = 1, k2z = 1.5;

		float k1x = 2, k2x = 2; // 0.25
		float k1y = 2, k2y = 2;
		float k1z = 1.5, k2z = 1; // look into tuning vertical controller for improvement

		// Observer gains
		float lambda1x = 5, lambda2x = 15, lambda3x = 3; //0.5
		float lambda1y = 5, lambda2y = 15, lambda3y = 3;
		float lambda1z = 5, lambda2z = 15, lambda3z = 3;

		//Error gains
		//float c1x = 0.75, c1y = 0.75, c1z = 1;

		float c1x = 1.5, c1y = 1.5, c1z = 1; // dont increase c1x and c1y up to 1.75, it doesnt like that!

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
		if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
		{ // Use PX4 module
			Uxx = 0.0f;
			Uyy = 0.0f;
			Uzz = 0.0f;
		}

		u_x = Uxx;
		u_y = Uyy;
		u_z = Uzz;

		// // x-dynamics observer  // x_hat += x_hat... --> integral
		// x_hat(0) = x_hat(0)  + lambda1x * cbrt(fabsf(x_tilde) * fabsf(x_tilde)) * sign(x_tilde) * dt + (dt*x_hat(1) + (dt*dt/2)*x_hat(2));
		// x_hat(1) = x_hat(1) +  lambda2x * cbrt(fabsf(x_tilde)) * sign(x_tilde) * dt + (dt*x_hat(2))+(u_x)*dt;
		// x_hat(2) = x_hat(2) + lambda3x * sign(x_tilde) * dt;
		// //x_hat(2) = x_hat(2) + lambda3x*sat_x*dt;  // Disturbance estimation with smoothing function
		// // y-dynamics observer
		// y_hat(0) = y_hat(0) + lambda1y * cbrt(fabsf(y_tilde) * fabsf(y_tilde)) * sign(y_tilde) * dt + (dt*y_hat(1) + (dt*dt/2)*y_hat(2));
		// y_hat(1) = y_hat(1) + lambda2y * cbrt(fabsf(y_tilde)) * sign(y_tilde) * dt + (dt*y_hat(2))+(u_y)*dt;
		// y_hat(2) = y_hat(2) + lambda3y * sign(y_tilde) * dt;
		// //y_hat(2) = y_hat(2) + lambda3y*sat_y*dt;  // Disturbance estimation with smoothing function
		// // z-dynamics observer**
		// z_hat(0) = z_hat(0) + lambda1z * cbrt(fabsf(z_tilde) * fabsf(z_tilde)) * sign(z_tilde) * dt + (dt*z_hat(1) + (dt*dt/2)*z_hat(2));
		// z_hat(1) = z_hat(1) + lambda2z * cbrt(fabsf(z_tilde)) * sign(z_tilde) * dt + (dt*y_hat(2)) + (u_z)*dt;
		// z_hat(2) = z_hat(2) + lambda3z * sign(z_tilde) * dt;
		// //z_hat(2) = z_hat(2) + lambda3z*sat_z*dt;  // Disturbance estimation with smoothing function

		// x-dynamics observer  // x_hat += x_hat... --> integral
		x_hat(0) = x_hat(0)  + (x_hat(1) * dt) + lambda1x * cbrt(fabsf(x_tilde) * fabsf(x_tilde)) * sign(x_tilde) * dt;
		x_hat(1) = x_hat(1) + (x_hat(2) * dt) + lambda2x * cbrt(fabsf(x_tilde)) * sign(x_tilde) * dt + (u_x)*dt;
		//x_hat(2) = x_hat(2) + lambda3x * sign(x_tilde) * dt;
		x_hat(2) = x_hat(2) + lambda3x*sat(x_tilde)*dt;  // Disturbance estimation with smoothing function
		// y-dynamics observer
		y_hat(0) = y_hat(0) + (y_hat(1) * dt) + lambda1y * cbrt(fabsf(y_tilde) * fabsf(y_tilde)) * sign(y_tilde) * dt;
		y_hat(1) = y_hat(1) + (y_hat(2) * dt) + lambda2y * cbrt(fabsf(y_tilde)) * sign(y_tilde) * dt + (u_y)*dt;
		//y_hat(2) = y_hat(2) + lambda3y * sign(y_tilde) * dt;
		y_hat(2) = y_hat(2) + lambda3y*sat(y_tilde)*dt;  // Disturbance estimation with smoothing function
		// z-dynamics observer**
		z_hat(0) = z_hat(0) + (z_hat(1) * dt) + lambda1z * cbrt(fabsf(z_tilde) * fabsf(z_tilde)) * sign(z_tilde) * dt;
		z_hat(1) = z_hat(1) + (z_hat(2) * dt) + lambda2z * cbrt(fabsf(z_tilde)) * sign(z_tilde) * dt + (u_z)*dt;
		//z_hat(2) = z_hat(2) + lambda3z * sign(z_tilde) * dt;
		z_hat(2) = z_hat(2) + lambda3z*sat(z_tilde)*dt;  // Disturbance estimation with smoothing function

				// Sliding manifold
		// Error definitions:
		Vector3f e_1 = _pos - _pos_sp; //x(0)-x_1d;
		Vector3f e_2 = _vel - vel_spi; //x(1) - x_2d;

		float sx = c1x * e_1(0) + (e_2(0));
		float sy = c1y * e_1(1) + (e_2(1));
		float sz = c1z * e_1(2) + (e_2(2));

		// Use saturation function instead of sign function
		float sat_sx = ControlMath::sat(sx);
		float sat_sy = ControlMath::sat(sy);
		float sat_sz = ControlMath::sat(sz);


		//Switching control terms
		sign_sx_int += k2x * sign(sx) * dt;
		sign_sy_int += k2y * sign(sy) * dt;
		sign_sz_int += k2z * sign(sz) * dt;

		sat_sxin += k2x*sat_sx * dt;
		sat_syin += k2y*sat_sy * dt;
		sat_szin += k2z*sat_sz * dt;

		// Define virtual controllers
		//float U_x = 0, U_y = 0, U_z = 0;

		// Uxx = 0.0f;
		// Uyy = 0.0f;
		// Uzz = 0.0f;

		// Uxx = -c1x * (e_2(0)) + x_ddot2di - k1x * sqrt(abs(sx)) * sign(sx) - sign_sx_int - x_hat(2);
		// Uyy = -c1y * (e_2(1)) + y_ddot2di - k1y * sqrt(abs(sy)) * sign(sy) - sign_sy_int - y_hat(2);
		// Uzz = -c1z * (e_2(2)) + z_ddot2di - k1z * sqrt(abs(sz)) * sign(sz) - sign_sz_int - z_hat(2);

		Uxx = -c1x * (e_2(0)) + x_ddot2di - k1x * sqrt(fabs(sx)) * sat_sx - sat_sxin;// - x_hat(2);
		Uyy = -c1y * (e_2(1)) + y_ddot2di - k1y * sqrt(fabs(sy)) * sat_sy - sat_syin;// - y_hat(2);
		Uzz = -c1z * (e_2(2)) + z_ddot2di - k1z * sqrt(fabs(sz)) * sat_sz - sat_szin;// - z_hat(2);

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
		pre_xdot_2di = vel_spi(0);
		pre_ydot_2di = vel_spi(1);
		pre_zdot_2di = vel_spi(2);
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

		//cout<<_pos(0)-x_hat(0)<<" "<<_vel(0)-x_hat(1)<<" "<<x_hat(2)<<" "<<e_2(0)<<endl;


#endif


	_accelerationControl();

//-----------------------------------------------------------------------------------
// These are some thrust calculations and conversions taken out of the velocity control 
// function that are needed:

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

}




//-------------------------------------------------------------------------------------

// Original PID position control module



void PositionControl::_positionControl()
{
	
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	pos_U = Vector3f(0.0f,0.0f,0.0f);
	// pos_U = Vector3f(vel_sp_position(0),vel_sp_position(1),0.0f); // use pid for x and y but not z
	
	// ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	ControlMath::addIfNotNanVector3f(_vel_sp, pos_U);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	//******* Use STSMC for vertical dynamics Controller ***************

	// vel_spi(2) = (_pos_sp(2) - pos_zi)/dt;

	// float z_ddot2di = (vel_spi(2) - pre_zdot_2di) / dt;  // Zero most of the time (has little influence) so ignore

	//float z_ddot2d = (_vel_sp(2) - pre_zdot_2d) / dt;


	// Controller gains
	// float k1z = 1.5, k2z = 2;

	// float c1z = 1;
	// working well
	// float k1z = 2.5, k2z = 3;

	// float c1z = 2;

	// float k1z = 5, k2z = 8; // 3.5 is good  k2 = 1.1*kappa  k1=sqrt(kappa) where kappa>0

	// float c1z = 5; // 7 worked amazing with 3 and 3.5

	//********* For x,y, and z controllers

	//Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving back and forth
	float k1x = 5.2, k2x = 6.2; // 0.1,0.2, 1 and 2 were good but wobbly  working: kx = 1 k2 = 1.45
	float k1y = 5.2, k2y = 6.2;  // k1 = 1  k2 = 1.55
	float k1z = 5.5, k2z = 6.5; // k1z=5.5 k2z=6.5 (non windy conditions)

	//Error gains
	//float c1x = 3, c1y = 3, c1z = 3;

	float c1x = 3, c1y = 3, c1z = 3; // 1.8, 4 was decent  --> Set cy higher then cx because y direction needs faster response for 8 trajectory
	

	//************************ Observer for use***********************
	// Observer gains
	// Observer gains
		// float lambda1x = 15, lambda2x = 16, lambda3x = 3; // non windy = 3 for lambda3 gains
		// float lambda1y = 15, lambda2y = 16, lambda3y = 3;
		// float lambda1z = 15, lambda2z = 16, lambda3z = 3;

	// Set the estimation (observer) initial conditions. Overwrties any elements of a Vector3f which are
		// NaN with zero
		ControlMath::setZeroIfNanVector3f(x_hat); /**<x-dynamic estimated states */
		ControlMath::setZeroIfNanVector3f(y_hat); /**<y-dynamic estimated states */
		ControlMath::setZeroIfNanVector3f(z_hat); /**<z-dynamic estimated states */


		// Define estimation errors
		float x_tilde = _pos(0) - x_hat(0);
		float y_tilde = _pos(1) - y_hat(0);
		float z_tilde = _pos(2) - z_hat(0);

		// Compute estimates with observer
		if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
		{ // Use PX4 module
			Uxx = 0.0f;
			Uyy = 0.0f;
			Uzz = 0.0f;
			a_desZ = 0.0f; // set this value too (not needed)
			U_v = 0.0f;
		}

		u_x = Uxx;
		u_y = Uyy;
		u_z = Uzz;

		// Adaptive observer gains
		// float M0 = 10;  // Soft bound you are setting
		// float N0 = 120; // Hardbound
		// float Lz_star = 5; // Previously known value i.e. leakage shift
		// float sigma0 = 0.2;

		// Leakage term update
	// 	if (!PX4_ISFINITE(Lx)|| !PX4_ISFINITE(Ly) || !PX4_ISFINITE(Lz)) {
	// 	Lx = 1.0f;
	// 	Ly = 1.0f;
	// 	Lz = 1.0f;
	// } else if (fabsf(Lz) < M0) {
	// 	wz = 0.0f;

	// } else if ((M0 <= fabsf(Lz)) && (fabsf(Lz)<= 2*M0)) {

	// 	wz = sigma0*((Lz/M0)-1);

	// }else{
	// 	wz = sigma0;

	// }


		// Mx += Kax*fabsf(x_tilde);
		// Mx += Kax*fabsf(x_tilde) - Kax*wx*(Mx-M0); // leakage

		// Lz += Kaz*fabsf(z_tilde);
		// Lz += Kaz*fabsf(z_tilde) - Kaz*wz*(Lz-Lz_star); // leakage

		// // Projection (hardbound)

		// if(fabsf(Lz)<= N0){
		// 	Lz+=Kaz*fabsf(z_tilde);
		// }else{
		// 	Lz += 0;
		// }

		// Deadzone decreasing adaptive gain
		//  thresh = 0.002; // Based on the noise of 0.001 std in matlab
		// thresh = 0.0025; // Based on the altitude covariance 0.0025
		float Kax = 1; // adjust convergence rate of adaptive gain
		float Kay = 1;
		float Kaz = 1.2;
		float threshx = 0.003; // 0.005
		float threshy = 0.003;
		float threshz = 0.0025;
		float mu = 0.1;    // Set higher for faster increase if you want
		if(fabs(Lx) >= 1.0f){
			Lx+=Kax*fabsf(x_tilde)*sign(fabsf(x_tilde)-threshx);
		}else{
			Lx+=mu;
		}

		if(fabs(Ly) >= 1.0f){
			Ly+=Kay*fabsf(y_tilde)*sign(fabsf(y_tilde)-threshy);
		}else{
			Ly+=mu;
		}

		if(fabs(Lz) >= 1.0f){
			Lz+=Kaz*fabsf(z_tilde)*sign(fabsf(z_tilde)-threshz);
		}else{
			Lz+=mu;
		}

		// use adaptive gains (uncomment)
		// lambda1x = 2.0f*powf(Mx,(1.0f/3.0f));
		// lambda2x = 1.5f*sqrtf(2.0f)*powf(Mx,((1.0f/2.0f)+(1.0f/6.0f)));
		// lambda3x = 1.1f*Mx;

		// y-dynamics
		lambda1xa = 3.0f*powf(Lx,(1.0f/3.0f));
		lambda2xa = 1.5f*powf(Lx,((1.0f/2.0f)));
		lambda3xa = 1.1f*Lx;

		// y-dynamics
		lambda1ya = 3.0f*powf(Ly,(1.0f/3.0f));
		lambda2ya = 1.5f*powf(Ly,((1.0f/2.0f)));
		lambda3ya = 1.1f*Ly;

		// z-dynamics
		lambda1za = 3.0f*powf(Lz,(1.0f/3.0f));
		lambda2za = 1.5f*powf(Lz,((1.0f/2.0f)));
		lambda3za = 1.1f*Lz;
		

		// // Other website: --> Mine is better
		// // z-dynamics observer**
		// z_hat(0) = z_hat(0) + (z_hat(1) * dt + (dt*dt/2)*z_hat(2)) + lambda1z * cbrt(fabsf(z_tilde) * fabsf(z_tilde)) * tanhf(z_tilde) * dt;
		// z_hat(1) = z_hat(1) + (z_hat(2) * dt) + lambda2z * cbrt(fabsf(z_tilde)) * tanhf(z_tilde) * dt + (u_z)*dt;
		// //z_hat(2) = z_hat(2) + lambda3z * sign(z_tilde) * dt;
		// //z_hat(2) = z_hat(2) + lambda3z*sat(z_tilde)*dt;  // Disturbance estimation with smoothing function
		// z_hat(2) = z_hat(2) + lambda3z*tanhf(z_tilde)*dt;  // Disturbance estimation with smoothing function

		float e_small = 1;

		// x-dynamics observer  // x_hat += x_hat... --> integral
		x_hat(0) = x_hat(0)  + (x_hat(1) * dt) + lambda1xa * cbrt(fabsf(x_tilde) * fabsf(x_tilde)) * sign(x_tilde) * dt;
		x_hat(1) = x_hat(1) + (x_hat(2) * dt) + lambda2xa * cbrt(fabsf(x_tilde)) * sign(x_tilde) * dt + (u_x)*dt;
		//x_hat(2) = x_hat(2) + lambda3x * sign(x_tilde) * dt;
		// x_hat(2) = x_hat(2) + lambda3x*sat(x_tilde/e_small)*dt;  // Disturbance estimation with smoothing function
		x_hat(2) = x_hat(2) + lambda3xa*tanhf(x_tilde/e_small)*dt;  // Disturbance estimation with smoothing function


		// y-dynamics observer
		y_hat(0) = y_hat(0) + (y_hat(1) * dt) + lambda1ya * cbrt(fabsf(y_tilde) * fabsf(y_tilde)) * sign(y_tilde) * dt;
		y_hat(1) = y_hat(1) + (y_hat(2) * dt) + lambda2ya * cbrt(fabsf(y_tilde)) * sign(y_tilde) * dt + (u_y)*dt;
		//y_hat(2) = y_hat(2) + lambda3y * sign(y_tilde) * dt;
		// y_hat(2) = y_hat(2) + lambda3ya*sat(y_tilde/e_small)*dt;  // Disturbance estimation with smoothing function
		y_hat(2) = y_hat(2) + lambda3ya*tanhf(y_tilde/e_small)*dt;  // Disturbance estimation with smoothing function

		// Mine:
		// z_hat(0) = z_hat(0) + (z_hat(1) * dt) + lambda1z * cbrt(fabsf(z_tilde) * fabsf(z_tilde)) * tanhf(z_tilde) * dt;
		// z_hat(1) = z_hat(1) + (z_hat(2) * dt) + lambda2z * cbrt(fabsf(z_tilde)) * tanhf(z_tilde) * dt + (u_z)*dt;
		// //z_hat(2) = z_hat(2) + lambda3z * sign(z_tilde) * dt;
		// //z_hat(2) = z_hat(2) + lambda3z*sat(z_tilde)*dt;  // Disturbance estimation with smoothing function
		// z_hat(2) = z_hat(2) + lambda3z*tanhf(z_tilde)*dt;  // Disturbance estimation with smoothing function
		
		z_hat(0) = z_hat(0) + (z_hat(1) * dt) + lambda1za * cbrtf(fabsf(z_tilde) * fabsf(z_tilde)) * sign(z_tilde) * dt;
		z_hat(1) = z_hat(1) + (z_hat(2) * dt) + lambda2za * cbrtf(fabsf(z_tilde)) * sign(z_tilde) * dt  - (Uzz)*dt; //CONSTANTS_ONE_G*dt; --> Dont use this, as Uzz is supposed to have g to cancel out but Uzz is 0 initially so it messes things up
		// z_hat(2) = z_hat(2) + lambda3za * sign(z_tilde) * dt;
		//z_hat(2) = z_hat(2) + lambda3z*sat(z_tilde)*dt;  // Disturbance estimation with smoothing function
		z_hat(2) = z_hat(2) + lambda3za*tanhf(z_tilde/e_small)*dt;  // Disturbance estimation with smoothing function


	//********************************************************************************

	

	// Sliding manifold
	// Error definitions:
	Vector3f e_1 = _pos - _pos_sp; //x(0)-x_1d;
	//Vector3f e_2 = _vel - vel_spi; //x(1) - x_2d;
	// Vector3f e_2 = _vel - _vel_sp; //x(1) - x_2d;

	// float sx = c1x * e_1(0) + (e_2(0));
	// float sy = c1y * e_1(1) + (e_2(1));
	// float sz = c1z * e_1(2) + (e_2(2));

	float r = 0.8;

	if (!PX4_ISFINITE(e1_int)) {
		// Setpoint NAN, take addition
		// e1_int = e_1(2) * dt;
		   e1_int = 0.0f;
	} else {
		// No NAN, add to integral
		e1_int +=  r*e_1(2) * dt;
	}
	// limit thrust integral
	e1_int = math::min(fabsf(e1_int), CONSTANTS_ONE_G) * sign(e1_int);

	// float sz = c1z*e_1(2) + (z_hat(1)-_vel_sp(2)) + e1_int;

	// float sz = c1z*e_1(2) + (z_hat(1)-_vel_sp(2)) - _vel_int(2);

	// Use observer for z-dynamics
	float sx = c1x * e_1(0) + (x_hat(1)-_vel_sp(0));
	// float sx = c1x * e_1(0) + (x_hat(1)-_vel_sp(0)) - _vel_int(0);
	float sy = c1y * e_1(1) + (y_hat(1)-_vel_sp(1));
	float sz = c1z * e_1(2) + (z_hat(1)-_vel_sp(2));
	// float sz = c1z*e_1(2) + (z_hat(1)-vel_spi(2));

	// Use saturation function instead of sign function
	// float sat_sx = ControlMath::sat(sx);
	// float sat_sy = ControlMath::sat(sy);
	// float sat_sz = ControlMath::sat(sz);
	// Use hyperbolic tan function tanh(x) to approximate sign function.
	float hyp_sx = tanhf(sx);
	float hyp_x_tilde = tanhf(x_tilde);

	float hyp_sy = tanhf(sy);
	float hyp_y_tilde = tanhf(y_tilde);

	float hyp_sz = tanhf(sz);
	float hyp_z_tilde = tanhf(z_tilde);


	// //Switching control terms
	sign_sx_int += k2x * sign(sx) * dt;
	// sign_sy_int += k2y * sign(sy) * dt;
	 sign_sz_int += k2z * sign(sz) * dt;
	// sat_sxin += sat_sx * dt;
	// sat_syin += sat_sy * dt;
	// sat_szin += sat_sz * dt;
	

	// guess the others work because of sign function? I am not sure
	if (!PX4_ISFINITE(hyp_sz_int)|| !PX4_ISFINITE(hyp_sy_int) || !PX4_ISFINITE(hyp_sx_int)) {
		// Setpoint NAN, take addition
		hyp_sx_int = k2x * hyp_sx * dt;
		hyp_sy_int = k2y * hyp_sy * dt;
		hyp_sz_int = k2z * hyp_sz * dt;
	}
	 else {
		// No NAN, add to integral
		hyp_sx_int += k2x * hyp_sx * dt;
		hyp_sy_int += k2y * hyp_sy * dt;
		hyp_sz_int += k2z * hyp_sz * dt;
	}
	
	// Define virtual controllers
	//float U_x = 0, U_y = 0, U_z = 0;

	// Uxx = 0.0f;
	// Uyy = 0.0f;
	// Uzz = 0.0f;

	//Uzz = -c1z * (e_2(2)) + z_ddot2di - k1z * sqrt(fabs(sz)) * sign(sz) - sign_sz_int;
	//Uzz = -c1z * (e_2(2)) - k1z * sqrt(fabs(sz)) * sign(sz) - sign_sz_int;
	//Uzz = -c1z * (e_2(2)) - k1z * sqrt(fabs(sz)) * sat_sz - k2z*sat_szin;
	//****************************************************************
	// Uxx = -c1x * (e_2(0)) - k1x * sqrt(fabs(sx)) * hyp_sx - hyp_sx_int;
	// Uyy = -c1y * (e_2(1)) - k1y * sqrt(fabs(sy)) * hyp_sy - hyp_sy_int;
	// Uzz = -c1z * (e_2(2)) - k1z * sqrt(fabs(sz)) * hyp_sz - hyp_sz_int;
	//****************************************************************
	//Uzz = - k1z * sqrt(fabs(sz)) * hyp_sz - hyp_sz_int;

	//************* Observer Controllers*************
	ControlMath::setZeroIfNanVector3f(acc_ff);
	// Uxx = -c1x * x_hat(1) + c1x * _vel_sp(0) - x_hat(2) - lambda2x * cbrt(fabsf(x_tilde)) * hyp_x_tilde - k1x * sqrt(fabs(sx)) * hyp_sx - hyp_sx_int + _gain_vel_i(0)*(_vel_sp(0) - _vel(0));;
	Uxx = -c1x * x_hat(1) + c1x * _vel_sp(0) - x_hat(2) - lambda2xa * cbrt(fabsf(x_tilde)) * hyp_x_tilde - k1x * sqrt(fabs(sx)) * hyp_sx - hyp_sx_int; //+ xddot = _acc_sp(0) --> feedforward component
	Uyy = -c1y * y_hat(1) + c1y * _vel_sp(1) - y_hat(2) - lambda2ya * cbrt(fabsf(y_tilde)) * hyp_y_tilde - k1y * sqrt(fabs(sy)) * hyp_sy - hyp_sy_int;  // Provided from mavros

	// Set Uzz properly in the NED frame according to my paper
	Uzz = c1z * z_hat(1) - c1z * _vel_sp(2) + z_hat(2) + lambda2za * cbrt(fabsf(z_tilde)) * hyp_z_tilde + k1z * sqrt(fabs(sz)) * hyp_sz + hyp_sz_int;// + CONSTANTS_ONE_G;  
	//a_desZ = Uzz+ CONSTANTS_ONE_G; // desired acceleration (not needed just illustrating model understanding) gravity is taken care of in acceleration function

	//*********************************************************************
	//Uzz = -c1z*z_hat(1) + c1z*_vel_sp(2) - z_hat(2) - lambda2z*cbrt(fabsf(z_tilde))*sign(z_tilde) - k1z*sqrt(fabs(sz))*sign(sz) - sign_sz_int;
	
	// Uzz = -c1z*z_hat(1) + c1z*_vel_sp(2) - z_hat(2) - lambda2z*cbrt(fabsf(z_tilde))*hyp_z_tilde - k1z*sqrt(fabs(sz))*hyp_sz - hyp_sz_int + _gain_vel_i(2)*(_vel_sp(2) - _vel(2));
	
	// Uzz = -c1z*z_hat(1) + c1z*_vel_sp(2) - z_hat(2) - lambda2z*cbrt(fabsf(z_tilde))*hyp_z_tilde - k1z*sqrt(fabs(sz))*hyp_sz - hyp_sz_int -r*e_1(2);


	if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
	{ // Use PX4 module
		Uxx = 0.0f;
		Uyy = 0.0f;
		Uzz = 0.0f;
	}
	


	// if (!PX4_ISFINITE(Uzz))
	// { // Use PX4 module
	// 	Uzz = 0.0f;
	// }

	//cout<<_vel_sp(0)<<" "<<_vel_sp(1)<<" "<<_vel_sp(2)<<" "<<!PX4_ISFINITE(_vel_sp(0))<<endl;

	// Save references to previous references:
	//pre_zdot_2d = _vel_sp(2);
	
	// Save references to previous references for numerical calculations:
	// pos_zi = _pos_sp(2);

	// Not needed anymore:
	// pre_zdot_2di = vel_spi(2);

	//pre_zdot_2d = _vel_sp(2);

	//cout << _pos(2)<<""<<_pos_sp(2)<<""<<_vel(2)<<""<<vel_spi(2)<<endl;
	//cout<<"SMC"<<" "<<_vel_sp(0)<<endl;


	// No control input from setpoints or corresponding states which are NAN
	//ControlMath::addIfNotNanVector3f(_acc_sp, U); // --> _acc_sp = _acc_sp + U  --> U becomes set point to be converted to a thrust and attitude

	//*******************************************************************

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
	//cout<<_vel(0)<<" "<<_vel_sp(0)<<" "<<vel_error(0)<<endl;

	// Ground effect compensation
	// from clover4_physics.xacro rotor radius = 0.06m

	if (((_pos(3)/r_rotor)> 0.5f) &&((_pos(3)/r_rotor)< 4.0f)) {

		U_v = (Uzz)*(1-tau*(r_rotor/(powf(4*_pos(2),((2.0f)))))); // Ground effect adds thrust so need to lower input
		
	}
	 else {
		
		U_v = Uzz;
	}

	

	// STSMC
	// U = Vector3f(acc_sp_velocity(0), acc_sp_velocity(1), -(Uzz)); // get rid of gravity term from controller as it is taken care of/implemented in the acceleration function
	// supposed to be a_des/|a_des| where gravity is included although PX4 uses just gravity in z_b for physical constraint reasons. i.e due to cancellations you dont
	// need gravity anywhere (in observer or controller) but just keeping it there to help understand the math model behind things. negative Uzz as I have calculated positive value for 
	// Uzz (how model paper stuff worked out where negative was included there). So put the negative here and the collective thrust calculates assuming this is negative i.e correct direction
	U = Vector3f(Uxx, Uyy, -(Uzz));
	// U = Vector3f(Uxx, Uyy, Uzz);
	hold = acc_sp_velocity;
	
	// No control input from setpoints or corresponding states which are NAN
	// ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);
	
	//STSMC
	ControlMath::addIfNotNanVector3f(_acc_sp, U); // --> _acc_sp = _acc_sp + U  --> U becomes set point to be converted to a thrust and attitude
	//cout<<_acc_sp(2)<<" "<<_pos(0)<<" "<<_pos_sp(0)<<" "<<_pos(0)-_pos_sp(0)<<" "<<_hover_thrust<<endl;
	//cout<<_acc_sp(2)<<" "<<Uzz<<" "<<sz<<" "<<hyp_sz<<" "<<hyp_sz_int<<endl;
	//cout<<_acc_sp(2)<<" "<<_hover_thrust<<" "<<z_hat(2)<<" "<<_pos(2)<<" "<<z_hat(0)<<" "<<_vel(2)<<" "<<z_hat(1)<<endl;
	// cout<<Uzz<<" "<<U_v<<" "<<" "<<(1-tau*(r_rotor/(powf(4*_pos(2),((2.0f))))))<<endl;
	
	_accelerationControl();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.0f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.0f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0;

	if (thrust_max_xy_squared > 0) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
	// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
	const Vector2f acc_sp_xy_limited = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
	const float arw_gain = 2.f / _gain_vel_p(0);
	vel_error.xy() = Vector2f(vel_error) - (arw_gain * (Vector2f(_acc_sp) - acc_sp_xy_limited));

	// Make sure integral doesn't get NAN
	ControlMath::setZeroIfNanVector3f(vel_error);
	// Update integral part of velocity control
	_vel_int += vel_error.emult(_gain_vel_i) * dt;

	// limit thrust integral
	_vel_int(2) = math::min(fabsf(_vel_int(2)), CONSTANTS_ONE_G) * sign(_vel_int(2));

	
}


void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized(); // Z_B(desired) not actual Z_B--> get actual Z_B from attitude measurements probably (need it for observer... you actually dont due to cancellation)
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;  // technically _acc_sp(2) is supposed to be negative -_acc_sp(2)
	// i.e collective thrust upwards (negative body direction). Although it worked before because I have uz as negative in the controller definition. Now I have it positive (per derivation)
	// and have to add the negative in when I add it to the acceleration setpoint.
	// cout<<_acc_sp(2)<<" "<<_hover_thrust<<" "<<collective_thrust<<" "<<_acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G)<<endl;
	// Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust; // Mass normalized collective thrust (desired/setpoint not current) so thr_sp[2] should be input vertical thrust
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

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

	// Set observer log variables
	local_position_setpoint.xhat0 = x_hat(0);
	local_position_setpoint.xhat1 = x_hat(1);
	local_position_setpoint.xhat2 = x_hat(2);
	local_position_setpoint.xhatep = fabsf(x_hat(0)-_pos(0));
	local_position_setpoint.xhatev = fabsf(x_hat(1)-_vel(0));
	local_position_setpoint.exp = fabsf(_pos(0)-_pos_sp(0));

	local_position_setpoint.yhat0 = y_hat(0);
	local_position_setpoint.yhat1 = y_hat(1);
	local_position_setpoint.yhat2 = y_hat(2);
	local_position_setpoint.yhatep = fabsf(y_hat(0)-_pos(1));
	local_position_setpoint.yhatev = fabsf(y_hat(1)-_vel(1));
	local_position_setpoint.eyp = fabsf(_pos(1)-_pos_sp(1));


	local_position_setpoint.zhat0 = z_hat(0);
	local_position_setpoint.zhat1 = z_hat(1);
	local_position_setpoint.zhat2 = z_hat(2);
	local_position_setpoint.zhatep = fabsf(z_hat(0)-_pos(2));
	local_position_setpoint.zhatev = fabsf(z_hat(1)-_vel(2));
	local_position_setpoint.ezp = fabsf(_pos(2)-_pos_sp(2));

	local_position_setpoint.lx = Lx;
	local_position_setpoint.lambda1xa = lambda1xa;
	local_position_setpoint.lambda2xa = lambda2xa;
	local_position_setpoint.lambda3xa = lambda3xa;

	local_position_setpoint.ly = Ly;
	local_position_setpoint.lambda1ya = lambda1ya;
	local_position_setpoint.lambda2ya = lambda2ya;
	local_position_setpoint.lambda3ya = lambda3ya;

	local_position_setpoint.lz = Lz;
	local_position_setpoint.lambda1za = lambda1za;
	local_position_setpoint.lambda2za = lambda2za;
	local_position_setpoint.lambda3za = lambda3za;

}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
