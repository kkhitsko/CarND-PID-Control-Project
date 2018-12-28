#include "PID.h"
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():
        p_error(0), i_error(0), d_error(0)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    best_error = std::numeric_limits<double>::max();

    dp = { Kp*0.1, Ki*0.1, Kd*0.1 };
}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    printf("p: %f; i: %f; d: %f\n", p_error, i_error, d_error );
}

double PID::TotalError() {
    double sum = -Kp*p_error -Ki*i_error - Kd*d_error;
    if ( sum > 1.0 ) sum = 1.0;
    if ( sum < -1.0 ) sum = -1.0;
    return sum;
}

