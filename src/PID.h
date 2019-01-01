#ifndef PID_H
#define PID_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>

class PID {
private:
    double best_error;
    int iteration_step;

    std::vector<double> dp;

    bool twiddle_optimization;

    std::ofstream log_file;

    std::ofstream opt_file;

    bool firtsTimeAdded;
    bool increaseParam;

    int param_idx = 2;

    double cur_cte;

    double total_error;

    int twiddle_period = 100;

    bool is_optimized;

    /* Counter of skip optiziation to detect when we should stop twiddle algorithm */
    int effectlessOptimizationCounter;

    int scaleFactorCounter;

protected:
    void Twiddle( );

    void UpdateParams( double addition, int param );

    std::string paramToStr( int param_idx );

    void scaleTwiddleParams( );

public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double sum_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double min_value;

  double max_value;

  int accumulateStepsNum;
  int decideStepsNum;

  double twiddle_factor;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double factor, const std::string &logfile, const std::string &optimizefile, bool twiddle_optimization = false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  bool isOptimized();

  void enableOptimization();

};

#endif /* PID_H */
