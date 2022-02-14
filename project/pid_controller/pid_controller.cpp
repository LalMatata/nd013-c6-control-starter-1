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

//TODO: Initialize PID coefficients (and errors, if needed)

    coef_p = Kpi;
    coef_i = Kii;
    coef_d = Kdi;

    error_p = 0.0;
    error_i = 0.0;
    error_d = 0.0;

    lim_max = output_lim_maxi;
    lim_min = output_lim_mini;
}


void PID::UpdateError(double cte) {

//TODO: Update PID errors based on cte.

    error_p = cte;
    error_i = cte + error_i;
    error_d = cte - error_p;

}

double PID::TotalError() {
    /**
    * TODO: Calculate and return the total error
     * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
    */

    double control = (coef_p * error_p) - (coef_i * error_i) - (coef_d * error_d);
    control= max(min(control,lim_max), lim_min);
    return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
    /**
    * TODO: Update the delta time with new value
    */
    delt = new_delta_time;
    return delt;
}