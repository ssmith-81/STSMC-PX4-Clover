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
#include <float.h>
#include <mathlib/mathlib.h>
#include <ControlMath.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

#define STSMC true // turn controller on
using namespace ControlMath;

PositionControl::PositionControl(ModuleParams *parent) :
	ModuleParams(parent)
{}

void PositionControl::updateState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::_setCtrlFlag(bool value)
{
	for (int i = 0; i <= 2; i++) {
		_ctrl_pos[i] = _ctrl_vel[i] = value;
	}
}

bool PositionControl::updateSetpoint(const vehicle_local_position_setpoint_s &setpoint)
{
	// by default we use the entire position-velocity control-loop pipeline (flag only for logging purpose)
	_setCtrlFlag(true);

	_pos_sp = Vector3f(setpoint.x, setpoint.y, setpoint.z);
	_vel_sp = Vector3f(setpoint.vx, setpoint.vy, setpoint.vz);
	_acc_sp = Vector3f(setpoint.acceleration);
	_thr_sp = Vector3f(setpoint.thrust);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
	bool mapping_succeeded = _interfaceMapping();

	// If full manual is required (thrust already generated), don't run position/velocity
	// controller and just return thrust.
	_skip_controller = PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))
			   && PX4_ISFINITE(_thr_sp(2));

	return mapping_succeeded;
}

void PositionControl::generateThrustYawSetpoint(const float dt)
{
	if (_skip_controller) {

		// Already received a valid thrust set-point.
		// Limit the thrust vector.
		float thr_mag = _thr_sp.length();

		if (thr_mag > _param_mpc_thr_max.get()) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_thr_max.get();

		} else if (thr_mag < _param_mpc_manthr_min.get() && thr_mag > FLT_EPSILON) {
			_thr_sp = _thr_sp.normalized() * _param_mpc_manthr_min.get();
		}

		// Just set the set-points equal to the current vehicle state.
		_pos_sp = _pos;
		_vel_sp = _vel;
		_acc_sp = _acc;

	} else {
		//_positionController();         //Turn these off, not using PID anymore
		//_velocityController(dt);      // Not using this velocity module anymore
		SMC_control(dt);              // Use SMC controller
	}
}

bool PositionControl::_interfaceMapping()
{
	// if nothing is valid, then apply failsafe landing
	bool failsafe = false;

	// Respects FlightTask interface, where NAN-set-points are of no interest
	// and do not require control. A valid position and velocity setpoint will
	// be mapped to a desired position setpoint with a feed-forward term.
	// States and setpoints which are integrals of the reference setpoint are set to 0.
	// For instance: reference is velocity-setpoint -> position and position-setpoint = 0
	//               reference is thrust-setpoint -> position, velocity, position-/velocity-setpoint = 0
	for (int i = 0; i <= 2; i++) {

		if (PX4_ISFINITE(_pos_sp(i))) {
			// Position control is required

			if (!PX4_ISFINITE(_vel_sp(i))) {
				// Velocity is not used as feedforward term.
				_vel_sp(i) = 0.0f;
			}

			// thrust setpoint is not supported in position control
			_thr_sp(i) = NAN;

			// to run position control, we require valid position and velocity
			if (!PX4_ISFINITE(_pos(i)) || !PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_vel_sp(i))) {

			// Velocity controller is active without position control.
			// Set integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_ctrl_pos[i] = false; // position control-loop is not used

			// thrust setpoint is not supported in velocity control
			_thr_sp(i) = NAN;

			// to run velocity control, we require valid velocity
			if (!PX4_ISFINITE(_vel(i))) {
				failsafe = true;
			}

		} else if (PX4_ISFINITE(_thr_sp(i))) {

			// Thrust setpoint was generated from sticks directly.
			// Set all integral states and setpoints to 0
			_pos_sp(i) = _pos(i) = 0.0f;
			_vel_sp(i) = _vel(i) = 0.0f;
			_ctrl_pos[i] = _ctrl_vel[i] = false; // position/velocity control loop is not used

			// Reset the Integral term.
			_thr_int(i) = 0.0f;
			// Don't require velocity derivative.
			_vel_dot(i) = 0.0f;

		} else {
			// nothing is valid. do failsafe
			failsafe = true;
		}
	}

	// ensure that vel_dot is finite, otherwise set to 0
	if (!PX4_ISFINITE(_vel_dot(0)) || !PX4_ISFINITE(_vel_dot(1))) {
		_vel_dot(0) = _vel_dot(1) = 0.0f;
	}

	if (!PX4_ISFINITE(_vel_dot(2))) {
		_vel_dot(2) = 0.0f;
	}

	if (!PX4_ISFINITE(_yawspeed_sp)) {
		// Set the yawspeed to 0 since not used.
		_yawspeed_sp = 0.0f;
	}

	if (!PX4_ISFINITE(_yaw_sp)) {
		// Set the yaw-sp equal the current yaw.
		// That is the best we can do and it also
		// agrees with FlightTask-interface definition.
		if (PX4_ISFINITE(_yaw)) {
			_yaw_sp = _yaw;

		} else {
			failsafe = true;
		}
	}

	// check failsafe
	if (failsafe) {
		// point the thrust upwards
		_thr_sp(0) = _thr_sp(1) = 0.0f;
		// throttle down such that vehicle goes down with
		// 70% of throttle range between min and hover
		_thr_sp(2) = -(_param_mpc_thr_min.get() + (_param_mpc_thr_hover.get() - _param_mpc_thr_min.get()) * 0.7f);
		// position and velocity control-loop is currently unused (flag only for logging purpose)
		_setCtrlFlag(false);
	}

	return !(failsafe);
}

void PositionControl::SMC_control(const float &dt)
{
 // Calculate acceleration of desired setpint for sliding mode controller:
 // Use this when a velocity setpoint is given
 float x_ddot2d = (_vel_sp(0) - pre_xdot_2d) / dt; // define pre values in hpp file as zero
 float y_ddot2d = (_vel_sp(1) - pre_ydot_2d) / dt;
 float z_ddot2d = (_vel_sp(2) - pre_zdot_2d) / dt;
 // Use this when only a position setpoint is given (numerically calculate velocity setpoint)
 vel_spi(0) = (_pos_sp(0)-pos_xi)/dt;
 vel_spi(1) = (_pos_sp(1)-pos_yi)/dt;
 vel_spi(2) = (_pos_sp(2)-pos_zi)/dt;

 float x_ddot2di = (vel_spi(0) - pre_xdot_2di) / dt; // define pre values in hpp file as zero
 float y_ddot2di = (vel_spi(1) - pre_ydot_2di) / dt;
 float z_ddot2di = (vel_spi(2) - pre_zdot_2di) / dt;
#if STSMC

 if (!PX4_ISFINITE(_vel_sp(0)) || !PX4_ISFINITE(_vel_sp(1)) || !PX4_ISFINITE(_vel_sp(2)))
 {
     // Position controller gains

 // float k1x = 3, k2x = 4;
 // float k1y = 3, k2y = 3;
 // float k1z = 4, k2z = 3;
 // Tuning notes:
 // Increasing k1 makes it more responsive in the plane but also very jittery once it is around like 2

 //Increasing k2x or k2y makes it wobble heavily in that plane so it will keep moving bck and forth
 float k1x = 1, k2x = 0.25; // 0.25
 float k1y = 1, k2y = 0.25;
 float k1z = 3, k2z = 0.8;

 //Error gains
 //float c1x = 3, c1y = 3, c1z = 3;

 float c1x = 2, c1y = 2, c1z = 1;

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


 Uxx = -c1x * (e_2(0)) + x_ddot2di - k1x * sqrt(fabsf(sx)) * sign(sx) - sign_sx_int;
 Uyy = -c1y * (e_2(1)) + y_ddot2di - k1y * sqrt(fabsf(sy)) * sign(sy) - sign_sy_int;
 Uzz = -c1z * (e_2(2)) + z_ddot2di - k1z * sqrt(fabsf(sz)) * sign(sz) - sign_sz_int;

 if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
 { // Use PX4 module
     Uxx = 0.0f;
     Uyy = 0.0f;
     Uzz = 0.0f;
 }

 //cout<<e_2(2)<<" "<<z_dot2d<<" "<<sz<<" "<<sign_sz_int<<" "<<Uzz<<" "<<Uyy<<" "<<Uxx<<endl;

 // Save references to previous references:
 pre_xdot_2di = vel_spi(0);
 pre_ydot_2di = vel_spi(1);
 pre_zdot_2di = vel_spi(2);
 pos_xi = _pos_sp(0);
 pos_yi = _pos_sp(1);
 pos_zi = _pos_sp(2);
 //cout << _pos(0)<<" "<<_pos_sp(0)<<" "<<_vel(0)<<" "<<vel_spi(0)<<endl;
 //cout << _pos(2)<<" "<<_pos_sp(2)<<" "<<_vel(2)<<" "<<vel_spi(2)<<endl;

 // Gather virtual control inputs into a vector
 U = Vector3f(Uxx, Uyy, Uzz);

 // Add these virtual control inputs to the acceleration setpoint (this is what the velocity PID controller produces)
 // this setpoint is then converted to a thrust and attitude setpoint. This is what the non-linear
 // decoupling equations are supposed to do, but PX4 takes advantage of quadcopter dynamics being differentially flat and use other methods.
 }
 else
 {
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

 Uxx = -c1x * (e_2(0)) + x_ddot2d - k1x * sqrt(fabsf(sx)) * sign(sx) - sign_sx_int;
 Uyy = -c1y * (e_2(1)) + y_ddot2d - k1y * sqrt(fabsf(sy)) * sign(sy) - sign_sy_int;
 Uzz = -c1z * (e_2(2)) + z_ddot2d - k1z * sqrt(fabsf(sz)) * sign(sz) - sign_sz_int;

 if (!PX4_ISFINITE(Uxx) || !PX4_ISFINITE(Uyy) || !PX4_ISFINITE(Uzz))
 { // Use PX4 module
     Uxx = 0.0f;
     Uyy = 0.0f;
     Uzz = 0.0f;
 }

 //cout<<e_2(2)<<" "<<z_dot2d<<" "<<sz<<" "<<sign_sz_int<<" "<<Uzz<<" "<<Uyy<<" "<<Uxx<<endl;

 // Save references to previous references:
 pre_xdot_2d = _vel_sp(0);
 pre_ydot_2d = _vel_sp(1);
 pre_zdot_2d = _vel_sp(2);
 //cout << _pos(2)<<_pos_sp(2)<<_vel(2)<<_vel_sp(2)<<endl;

 // Gather virtual control inputs into a vector
 U = Vector3f(Uxx, Uyy, Uzz);

 // Add these virtual control inputs to the acceleration setpoint (this is what the velocity PID controller produces)
 // this setpoint is then converted to a thrust and attitude setpoint. This is what the non-linear
 // decoupling equations are supposed to do, but PX4 takes advantage of quadcopter dynamics being differentially flat and use other methods.
 }
#endif

//Acceleration control function from newer PX4 firmware
//Assume standard acceleration due to gravity in vertical direction for attitude generation
	//Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), CONSTANTS_ONE_G).normalized();
	float CONSTANTS_ONE_G = 9.81; // Gravitational constant
	Vector3f body_z = Vector3f(-U(0), -U(1), CONSTANTS_ONE_G).normalized();
	
	float _lim_tilt = _constraints.tilt;  // max tilt in radians
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	float _hover_thrust = _param_mpc_thr_hover.get();
	//Scale thrust assuming hover thrust produces standard gravity
	float collective_thrust = U(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	//Project thrust to planned body attitude
	collective_thrust /= (Vector3f(0, 0, 1).dot(body_z));
	//collective_thrust = math::min(collective_thrust, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;


//-----------------------------------------------------------------------------------
// These are some thrust calculations and conversions taken out of the velocity control
// function that are needed:

// The Thrust limits are negated and swapped due to NED-frame.
	float uMax = -_param_mpc_thr_min.get();
	float uMin = -_param_mpc_thr_max.get();

	// make sure there's always enough thrust vector length to infer the attitude
	uMax = math::min(uMax, -10e-4f);

	// Saturate thrust setpoint in D-direction.
	_thr_sp(2) = math::constrain(_thr_sp(2), uMin, uMax);

	
	// Thrust set-point in NE-direction is already provided. Only
	// scaling by the maximum tilt is required.
	float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
	_thr_sp(0) *= thr_xy_max;
	_thr_sp(1) *= thr_xy_max;
	
	
}

//-------------------------------------------------------------------------------------

// Original PID position control module

// void PositionControl::_positionController()
// {
// 	// P-position controller
// 	const Vector3f vel_sp_position = (_pos_sp - _pos).emult(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(),
// 					 _param_mpc_z_p.get()));
// 	_vel_sp = vel_sp_position + _vel_sp;

// 	// Constrain horizontal velocity by prioritizing the velocity component along the
// 	// the desired position setpoint over the feed-forward term.
// 	const Vector2f vel_sp_xy = ControlMath::constrainXY(Vector2f(vel_sp_position),
// 				   Vector2f(_vel_sp - vel_sp_position), _param_mpc_xy_vel_max.get());
// 	_vel_sp(0) = vel_sp_xy(0);
// 	_vel_sp(1) = vel_sp_xy(1);
// 	// Constrain velocity in z-direction.
// 	_vel_sp(2) = math::constrain(_vel_sp(2), -_constraints.speed_up, _constraints.speed_down);
// }

// void PositionControl::_velocityController(const float &dt)
// {
// 	// Generate desired thrust setpoint.
// 	// PID
// 	// u_des = P(vel_err) + D(vel_err_dot) + I(vel_integral)
// 	// Umin <= u_des <= Umax
// 	//
// 	// Anti-Windup:
// 	// u_des = _thr_sp; r = _vel_sp; y = _vel
// 	// u_des >= Umax and r - y >= 0 => Saturation = true
// 	// u_des >= Umax and r - y <= 0 => Saturation = false
// 	// u_des <= Umin and r - y <= 0 => Saturation = true
// 	// u_des <= Umin and r - y >= 0 => Saturation = false
// 	//
// 	// 	Notes:
// 	// - PID implementation is in NED-frame
// 	// - control output in D-direction has priority over NE-direction
// 	// - the equilibrium point for the PID is at hover-thrust
// 	// - the maximum tilt cannot exceed 90 degrees. This means that it is
// 	// 	 not possible to have a desired thrust direction pointing in the positive
// 	// 	 D-direction (= downward)
// 	// - the desired thrust in D-direction is limited by the thrust limits
// 	// - the desired thrust in NE-direction is limited by the thrust excess after
// 	// 	 consideration of the desired thrust in D-direction. In addition, the thrust in
// 	// 	 NE-direction is also limited by the maximum tilt.

// 	const Vector3f vel_err = _vel_sp - _vel;

// 	// Consider thrust in D-direction.
// 	float thrust_desired_D = _param_mpc_z_vel_p.get() * vel_err(2) +  _param_mpc_z_vel_d.get() * _vel_dot(2) + _thr_int(
// 					 2) - _param_mpc_thr_hover.get();

// 	// The Thrust limits are negated and swapped due to NED-frame.
// 	float uMax = -_param_mpc_thr_min.get();
// 	float uMin = -_param_mpc_thr_max.get();

// 	// make sure there's always enough thrust vector length to infer the attitude
// 	uMax = math::min(uMax, -10e-4f);

// 	// Apply Anti-Windup in D-direction.
// 	bool stop_integral_D = (thrust_desired_D >= uMax && vel_err(2) >= 0.0f) ||
// 			       (thrust_desired_D <= uMin && vel_err(2) <= 0.0f);

// 	if (!stop_integral_D) {
// 		_thr_int(2) += vel_err(2) * _param_mpc_z_vel_i.get() * dt;

// 		// limit thrust integral
// 		_thr_int(2) = math::min(fabsf(_thr_int(2)), _param_mpc_thr_max.get()) * math::sign(_thr_int(2));
// 	}

// 	// Saturate thrust setpoint in D-direction.
// 	_thr_sp(2) = math::constrain(thrust_desired_D, uMin, uMax);

// 	if (PX4_ISFINITE(_thr_sp(0)) && PX4_ISFINITE(_thr_sp(1))) {
// 		// Thrust set-point in NE-direction is already provided. Only
// 		// scaling by the maximum tilt is required.
// 		float thr_xy_max = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
// 		_thr_sp(0) *= thr_xy_max;
// 		_thr_sp(1) *= thr_xy_max;

// 	} else {
// 		// PID-velocity controller for NE-direction.
// 		Vector2f thrust_desired_NE;
// 		thrust_desired_NE(0) = _param_mpc_xy_vel_p.get() * vel_err(0) + _param_mpc_xy_vel_d.get() * _vel_dot(0) + _thr_int(0);
// 		thrust_desired_NE(1) = _param_mpc_xy_vel_p.get() * vel_err(1) + _param_mpc_xy_vel_d.get() * _vel_dot(1) + _thr_int(1);

// 		// Get maximum allowed thrust in NE based on tilt and excess thrust.
// 		float thrust_max_NE_tilt = fabsf(_thr_sp(2)) * tanf(_constraints.tilt);
// 		float thrust_max_NE = sqrtf(_param_mpc_thr_max.get() * _param_mpc_thr_max.get() - _thr_sp(2) * _thr_sp(2));
// 		thrust_max_NE = math::min(thrust_max_NE_tilt, thrust_max_NE);

// 		// Saturate thrust in NE-direction.
// 		_thr_sp(0) = thrust_desired_NE(0);
// 		_thr_sp(1) = thrust_desired_NE(1);

// 		if (thrust_desired_NE * thrust_desired_NE > thrust_max_NE * thrust_max_NE) {
// 			float mag = thrust_desired_NE.length();
// 			_thr_sp(0) = thrust_desired_NE(0) / mag * thrust_max_NE;
// 			_thr_sp(1) = thrust_desired_NE(1) / mag * thrust_max_NE;
// 		}

// 		// Use tracking Anti-Windup for NE-direction: during saturation, the integrator is used to unsaturate the output
// 		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
// 		float arw_gain = 2.f / _param_mpc_xy_vel_p.get();

// 		Vector2f vel_err_lim;
// 		vel_err_lim(0) = vel_err(0) - (thrust_desired_NE(0) - _thr_sp(0)) * arw_gain;
// 		vel_err_lim(1) = vel_err(1) - (thrust_desired_NE(1) - _thr_sp(1)) * arw_gain;

// 		// Update integral
// 		_thr_int(0) += _param_mpc_xy_vel_i.get() * vel_err_lim(0) * dt;
// 		_thr_int(1) += _param_mpc_xy_vel_i.get() * vel_err_lim(1) * dt;
// 	}
// }

void PositionControl::updateConstraints(const vehicle_constraints_s &constraints)
{
	_constraints = constraints;

	// For safety check if adjustable constraints are below global constraints. If they are not stricter than global
	// constraints, then just use global constraints for the limits.

	const float tilt_max_radians = math::radians(math::max(_param_mpc_tiltmax_air.get(), _param_mpc_man_tilt_max.get()));

	if (!PX4_ISFINITE(constraints.tilt)
	    || !(constraints.tilt < tilt_max_radians)) {
		_constraints.tilt = tilt_max_radians;
	}

	if (!PX4_ISFINITE(constraints.speed_up) || !(constraints.speed_up < _param_mpc_z_vel_max_up.get())) {
		_constraints.speed_up = _param_mpc_z_vel_max_up.get();
	}

	if (!PX4_ISFINITE(constraints.speed_down) || !(constraints.speed_down < _param_mpc_z_vel_max_dn.get())) {
		_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
	}

	if (!PX4_ISFINITE(constraints.speed_xy) || !(constraints.speed_xy < _param_mpc_xy_vel_max.get())) {
		_constraints.speed_xy = _param_mpc_xy_vel_max.get();
	}
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
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}

void PositionControl::updateParams()
{
	ModuleParams::updateParams();
}
