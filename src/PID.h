#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  double steer_value;
  double best_err ;

  // potential changes of params while twiddling  
  std::vector<double> dp;

  //PID parameters Kp, Ki, Kd
  char params[4];   
  int param_cnt; 
	
  int message_cnt;
  bool twiddle_tuned;
  double sum_dp;
  int improved;

  double error_sum;


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
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
  *  Get the steer values calculated
  */
  double getSteerValue();
  
  /*
  *  Invokes UpdateError and TotalError functions for twiddle functionality
  */
  void TwiddleRun(double cte);
  
  /*
  *  Returns the normalized cte over iterations
  */
  double ErrorSum();
  
  /*
  *   Implements twiddle algorithm for automatic tuning of PID controller params
  */
  void TwiddleTune(double tol);
};

#endif /* PID_H */

