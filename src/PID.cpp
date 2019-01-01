#include "PID.h"
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID():
        p_error(0), i_error(0), d_error(0), iteration_step(0), firtsTimeAdded(false),increaseParam(false),
        decideStepsNum(100),accumulateStepsNum(1500),sum_error(0),Kp(0.0),Ki(0.0),Kd(0.0),min_value(-1), max_value(1),
        best_error(0),twiddle_optimization(false), cur_cte(0), total_error(0), twiddle_factor(0.1)
{}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double factor, const std::string &logfile, const std::string &optimizefile, bool twiddle_opt) {

    twiddle_optimization = twiddle_opt;
    twiddle_factor = factor;

    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    best_error = std::numeric_limits<double>::max();

    dp = { Kp*twiddle_factor, Ki*twiddle_factor, Kd*twiddle_factor };

    log_file.open( logfile );
    if( !log_file.is_open() ) {
        abort();
    }

    opt_file.open( optimizefile );
    if( !opt_file.is_open() ) {
        abort();
    }

    total_error = 0;
    is_optimized = false;

    effectlessOptimizationCounter = 0;
    scaleFactorCounter = 0;

}

void PID::UpdateError(double cte) {
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;
    //printf("p: %f; i: %f; d: %f\n", p_error, i_error, d_error );

    cur_cte = cte*cte;

    iteration_step++;
    printf("Step: %d\n", iteration_step);
}

double PID::TotalError() {
    double sum = -Kp*p_error -Ki*i_error - Kd*d_error;
    sum_error = sum;
    log_file << sum << endl;
    if ( sum > max_value ) sum = max_value;
    if ( sum < min_value ) sum = min_value;

    if ( twiddle_optimization ) Twiddle( );



    return sum;
}

void PID::Twiddle( ) {

    int twiddleSteps = accumulateStepsNum + decideStepsNum;


    if (iteration_step % twiddleSteps > decideStepsNum) {
        total_error += cur_cte;
    }

    if( iteration_step % twiddleSteps == 0 ) {

        printf("Step %d. Total: %f. Best: %f; Twiddle params: p %f; i %f; d %f\n", iteration_step, total_error, best_error, Kp, Ki, Kd );
        opt_file << "Step: " << iteration_step << endl;
        opt_file << "Total error: " << total_error << "; Best error: "<< best_error << endl;

        if ( total_error < best_error ) {
            printf("Found new best error value: %f\n", total_error );
            opt_file << "Found new best error value: "<< total_error << endl;
            best_error = total_error;
            dp[param_idx] *= 1.1;
            /* Try to tune next parameter  */
            param_idx = ( param_idx + 1 ) % 3;
            firtsTimeAdded = false;
            increaseParam = false;

            effectlessOptimizationCounter = 0;
        }
        if ( !firtsTimeAdded && !increaseParam ) {
            opt_file << "First time of parameter " << paramToStr(param_idx) << " update " << endl;
            UpdateParams( dp[param_idx], param_idx );
            firtsTimeAdded = true;
        }
        else if ( firtsTimeAdded && !increaseParam ) {
            opt_file << "Increase parameter " << paramToStr(param_idx) << " but don't have min error" << endl;
            UpdateParams( -2*dp[param_idx], param_idx );
            increaseParam = true;
        }
        else {
            opt_file << "There is no effect while change " << paramToStr(param_idx) << " parameter. Return parameter value " << endl;
            UpdateParams( dp[param_idx], param_idx );
            dp[param_idx] *= 0.9;
            /* Try to tune next parameter  */
            param_idx = ( param_idx + 1 ) % 3;
            firtsTimeAdded = false;
            increaseParam = false;

            effectlessOptimizationCounter++;

            if ( effectlessOptimizationCounter > 3 ) {
                scaleTwiddleParams();
            }
        }
        total_error = 0;
    }
    if ( is_optimized ) {
        opt_file << "******************************" <<endl;
        opt_file << "Optimization complete!!! " <<endl;
        opt_file << "Kp: " << Kp << "; Ki: " << Ki << "; Kd: " << Kd << endl;
    }


}

void PID::UpdateParams( double addition, int param ) {
    opt_file << "Addition: " << addition << endl;
    switch ( param ) {
        case 0:
            Kp += addition;
            opt_file << "Tune param: " << paramToStr(param) << endl;
            break;
        case 1:
            Ki += addition;
            opt_file << "Tune param: " << paramToStr(param) << endl;
            break;
        case 2:
            Kd += addition;
            opt_file << "Tune param:" << paramToStr(param) << endl;
            break;
        default:
            break;
    }

    printf("Params after update: p %f; i %f; d %f\n", Kp, Ki, Kd );
    opt_file << "Kp: " << Kp << "; Ki: " << Ki << "; Kd: " << Kd << endl;
    opt_file << "------------------------" << endl;
}

std::string PID::paramToStr( int param_idx ) {
    std::string str = "";
    switch ( param_idx ) {
        case 0:
            str =  "proportional";
            break;
        case 1:
            str =  "integral";
            break;
        case 2:
            str = "derivative";
            break;
        default:
            break;
    }
    return str;
}

bool PID::isOptimized() {
    return is_optimized;
}

void PID::enableOptimization() {
    twiddle_optimization = true;
}

void PID::scaleTwiddleParams() {
    dp[0] *= 0.5;
    dp[1] *= 0.5;
    dp[2] *= 0.5;

    scaleFactorCounter++;
    effectlessOptimizationCounter = 0;
    param_idx = 2;

    opt_file << "--------Scale twiddle optimizations parameters----------------" << endl;

    if ( scaleFactorCounter == 4 ) {
        twiddle_optimization = false;
        is_optimized = true;
    }
}

