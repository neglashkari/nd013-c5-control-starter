/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  
  
 //defining coefficients
   kp = Kpi;
   kd = Kdi;
   ki = Kii;
  
   //initializing errors
   cte_prev = 0.0; //error
   cte_der = 0.0; //derivative of error
   cte_int = 0.0; //integral of error
   dt = 0.0;
   
   //define limits
   max_lim = output_lim_maxi;
   min_lim = output_lim_mini; 
  
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  
   cte_prev = cte;
   cte_der = dt > 0.0 ? (cte - cte_prev) / dt : 0.0; 
   cte_int += cte * dt;
  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
  

   double control = kp*cte_prev + kd*cte_der + ki*cte_int;
	
   if (control > max_lim) {
      control = max_lim;
   } else if (control < min_lim) {
      control = min_lim;
   }

   return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  
  dt = new_delta_time;
  
  return dt;
}
