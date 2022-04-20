#ifndef _PID_H_
#define _PID_H_

#include </home/clover/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PX4-Autopilot-master/src/lib/matrix/matrix/math.hpp>

using namespace matrix;

class SMC
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        float SMC_control(const Vector2f x, const float ref);
        Vector3f observer(const float U,  const Vector2f x);
        float sat(const float k);
        // Function to test the observer alone with the PID controller before testing STSMC

        // System dynamics functions
        Vector2f x_dynamics(const float U, const float i, const Vector3f XI);
        Vector2f y_dynamics(const Vector3f U, const Vector3f XI);
        Vector2f z_dynamics(const Vector3f U, const Vector3f XI);
        double PID(const Vector2f x, const float ref);

        Vector3f disturbances(const float i);
        float trajectory(const float i);
    

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        //~PID();

    //private:

        // States
	    matrix::Vector3f _pos = Vector3f(0,0,0); /**< current position */
	    matrix::Vector3f _vel = Vector3f(0,0,0); /**< current velocity */ //--> Estimated by observer
	    matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	    matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	    float _yaw{}; /**< current heading */

        matrix::Vector3f _pos_sp; /**< desired position */
        matrix::Vector3f _vel_sp; /**< velocity set point */
        matrix::Vector3f _vel_sp_dot; /**< derivative of velocity set point */
        // Set initial conditions for these setpoint values
        Vector3f _pos_sp_previous = Vector3f(0,0,0);
        Vector3f _vel_sp_previous = Vector3f(0,0,0);
        
        // STSMC controller definitions 
	
	    // Higher order sliding mode observer
        float Uxx{};
	    float Uyy{};
	    float Uzz{};
	    matrix::Vector3f x_hat = Vector3f(0,0,0);  /**<x-dynamic estimated state */
	    matrix::Vector3f y_hat = Vector3f(0,0,0);  /**<y-dynamic estimated state */
	    matrix::Vector3f z_hat= Vector3f(0,0,0);  /**<z-dynamic estimated state */
        float sign_sx_int{};
        float sign_sy_int{};
        float sign_sz_int{};
            

        // System Dynamics
        matrix::Vector2f x = Vector2f(0,0);
        matrix::Vector2f y = Vector2f(0,0);
        matrix::Vector2f z = Vector2f(0,0);

        // System disturbances
        matrix::Vector3f XI;

        // Desired trajectory definitions
        
            // Sine wave
            float ref;
            float pre_x_1d = 0;
            float pre_x_2d = 0;

        // PID
        float integral = 0;
        float Ki = 2;
        float Kp = 5;
        float Kd = 2;
        double pre_error = 0;

        // Saturation
        //float k{};
            
};

#endif