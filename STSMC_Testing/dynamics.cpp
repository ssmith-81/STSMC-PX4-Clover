#include <iostream>
#include <cmath>
#include <fstream>
// include the PX4 math library
#include </home/clover/PX4-Autopilot/src/modules/mc_pos_control/PositionControl/PX4-Autopilot-master/src/lib/matrix/matrix/math.hpp>
//#include <matrix/matrix/math.hpp>
//#include</home/clover/PX4-Autopilot/platforms/common/include/px4_platform_common/defines.h>
#include "controller.h"

using namespace std;
using namespace matrix;

float _dt = 0.02; // time step --> Global variable.

Vector2f SMC::x_dynamics(const float U, const float i, const Vector3f XI)
{
    
   if (isfinite(x(0)) == 0){
       x(0) = 0.0f;
    }
    
   if (isfinite(x(1)) == 0){
      x(1) = 0.0f;
    }
    //cout << x(1);
    //cout << x(0);
// x-system dynamics.
//x(0) = 0;
//x(1) = 0;

    
    x(0) = x(0) + (x(1))*_dt;
    x(1) = x(1) + (U)*_dt; //+ XI(0)*_dt; //+ sin(i); // + XI(0);

   // cout << x(0)<< endl;

    return x;

}

Vector3f SMC::disturbances(const float i)
    {
        // x-dynamic disturbance
        XI(0) = 0.6*sin(i);
        // y-dynamic disturbance
        XI(1) = 0.4*sin(i);
        // z-dynamic disturbance
        XI(2) = 0.2*sin(i);

        return XI;
    }


double SMC::PID(const Vector2f x, const float ref){

    // Calculate error

    double error = ref - x(0);

    // Proportional term
    double Pout = Kp*error;

    // Integral term
    integral += error*_dt;
    double Iout = Ki*integral;

    // Derivative term
    double derivative = (error - pre_error)/_dt;
    double Dout = Kd*derivative;

    // Calculate total output

    double output = Pout + Iout + Dout;

    // Save error to previous error

    pre_error = error;

   // cout<<error<<endl;

    return output;
}

float SMC::trajectory(const float i)
{
    #define circular_helix false
    #define figure_8 false
    #define sine_wave true
    #define constant false

    #if circular_helix
     float  a = 5.0, b = 0.4;
        _pos_sp(0) = a*cos(i);
        _pos_sp(1) = a*sin(i);
        _pos_sp(2) = b*i;

        return _pos_sp;
    #endif

    #if figure_8
    float  R = 5.0, H = 0.4;
        _pos_sp(0) = R*sin(i);
        _pos_sp(1) = R*sin(i)*cos(i);
        _pos_sp(2) = H;

        return _pos_sp;
    #endif

    #if sine_wave
    //cout<< i;
    ref = sin(i);
    //cout<<ref<<endl;
    return ref;

    #endif

    #if constant
    ref = 5.0f;

    return ref;
    
    #endif
}

float SMC::SMC_control(const Vector2f x, const float ref){

    // Position controller gains
		float k1x = 3, k2x = 4;
	

		// Observer gains
		float lambda1x = 5, lambda2x = 20, lambda3x = 9;
	
		//Error gains
		float c1x = 3;

         if (isfinite(Uxx) == 0){
                Uxx = 0.0f;
            }
        
        // position setpoint
        float x_1d = ref;
            // Velocity setpoint
            float x_2d = (x_1d - pre_x_1d)/_dt;
            float x_dot2d = (x_2d - pre_x_2d)/_dt;


    #define STSMC true
    #define STSMC_Observer false

    #if STSMC
    // Sliding manifold
			// Error definitions:
		    float e_1 = x(0)-x_1d;
			float e_2 = x(1) - x_2d;

        float sx = c1x*e_1 + (e_2);
    //Switching control terms
        sign_sx_int += k2x*sign(sx)*_dt;

    // Define virtual controllers
		float U_x = 0;
		
		U_x = -c1x*(e_2) + x_dot2d - k1x*sqrt(abs(sx))*sign(sx) - sign_sx_int;

        float U = U_x;

    // Save references to previous references:
    
        pre_x_1d = x_1d;
        pre_x_2d = x_2d;


        return U;

    #endif

    #if STSMC_Observer

    #endif 
    

    
}

Vector3f SMC::observer(const float U, const Vector2f x){

    // Observer gains
		float lambda1x = 5, lambda2x = 20, lambda3x = 9;

        float u_x = U;


// Define estimation errors
			float x_tilde = x(0) - x_hat(0);

                x_hat(0) = x_hat(0) + (x_hat(1)*_dt) + lambda1x*cbrt(fabsf(x_tilde)*fabsf(x_tilde))*sign(x_tilde)*_dt;
				x_hat(1) = x_hat(1) + (x_hat(2)*_dt) + lambda2x*cbrt(fabsf(x_tilde))*sign(x_tilde)*_dt + (u_x)*_dt;
				x_hat(2) = x_hat(2) + lambda3x*sign(x_tilde)*_dt;  // Disturbance estimation sat_x??
    return x_hat;
}
int main() {

    SMC SMC;
    ofstream data, meta, beta;
    data.open("datay.txt");
    meta.open("datax.txt");
    beta.open("data.txt");

    float U;
for (float i = 0; i < 10; i=i+0.02) {

    // Obtain reference trajectory
    SMC.ref = SMC.trajectory(i);

    // Obtain system disturbance
    SMC.XI = SMC.disturbances(i);


    // Define control input to the system dynamics

        //U = SMC.ref; // Constant input of 5
       // U = 5.0f - SMC.x(0); // Feed the feedback error into the system

       // U = SMC.PID(SMC.x,SMC.ref); // Implement PID control on the system dynamics
        U = SMC.SMC_control(SMC.x,SMC.ref);

        // Ensure the control input is finite, set to zero if it is not
         if (isfinite(U) == 0){
            U = 0.0f;
             }

        // x-dynamics
        SMC.x = SMC.x_dynamics(U,i,SMC.XI); // 


        //cout << i;
        //cout << x(0);
        //U = 5+x(0);
        cout<<U<<endl;
        //cout << U<<endl;

        // Observer testing function

       // SMC.x_hat = SMC.observer(U,SMC.x);

         // Store data in txt file
        meta<< i <<endl;
        data << SMC.x(0)<<endl;
        beta << U <<endl;
       // data << SMC.x_hat(2) <<endl;
        
         }
         data.close();
         meta.close();
         beta.close();
return 0;
   
}