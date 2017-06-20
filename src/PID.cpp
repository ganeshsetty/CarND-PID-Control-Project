#include "PID.h"
#include <uWS/uWS.h>
#define N 100

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;
	
	steer_value = 0;
	best_err = 1e10;
	error_sum = 0;

    //Potential changes of params for twiddle
	dp.push_back(0.1);
    dp.push_back(0.001);
    dp.push_back(1.05);

    //     
    params[0]= 'P';
    params[1] = 'I';
    params[2] = 'D';
    params[3] = '\0';    
	param_cnt = 0; 
	
	message_cnt = 0;
	twiddle_tuned = false;
	sum_dp = 0;
	improved  = 1;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	steer_value = -Kp*p_error -Ki*i_error -Kd*d_error;
	if( steer_value > 1) {
		steer_value = 1;
	}
	if(steer_value < -1) {
		steer_value = -1;
	}
	return steer_value;
}

double PID::getSteerValue() {
	return steer_value;
}

void PID::TwiddleRun(double cte) {
	UpdateError(cte);
	steer_value = TotalError();
	if(message_cnt > N) {
		error_sum += pow(cte,2) ;
	}
}

double PID::ErrorSum() {
	return error_sum/(message_cnt - N) ;
}


void PID::TwiddleTune(double tol) {
	double err = ErrorSum(); //save summation of cte for given number of iterations
	int iteration_cnt = message_cnt;
	error_sum = 0;
	message_cnt = 0;
    
    //std::cout << "inside twiddletune" << std::endl;
	
	sum_dp = dp[0] + dp[1] + dp[2];
	if( sum_dp < tol) twiddle_tuned = true;
	
	else {
		if(best_err > 1e8) {
			best_err = err;
					
			Kp += dp[0]; //First updation in tuning
			return;
		}
		i_error = 0;
		d_error = 0;
		p_error = 0;
	
		char param = params[param_cnt];
		param_cnt++;
		if(param_cnt ==3) param_cnt = 0;
		//std::cout << "error sum: "<< err << " best_err: " << best_err << std::endl; 	
		if(err < best_err && iteration_cnt > 200) {
            std::cout << "Before 1st switch" << std::endl;
            std::cout << "param : " << param << std::endl;
			best_err = err;
		
			switch(param) {
				case 'P' :
                    //std::cout << "1st switch inside param 'P'" << std::endl;
					dp[0] *= 1.1;
					break;
				case 'I' :
                    //std::cout << "1st switch inside param I " << std::endl;
					dp[1] *= 1.1;
					break;
				case 'D' :
                    //std::cout << "1st switch inside param D " << std::endl;
					dp[2] *= 1.1;
					break;
				default:
                    std::cout << "Invaid param" << std::endl;
			}
			improved = 1;
					 
		}
		else {
											
			if(improved == 1) improved = 0;
			else {
				switch(param) {
				case 'P' :
                    //std::cout << "2nd switch inside param P " << std::endl;
					Kp += dp[0];
					dp[0] *= 0.9;
					break;
				case 'I' :
                    //std::cout << " 2nd switch inside param I " << std::endl;
					Ki += dp[1];
					dp[1] *= 0.9;
					break;
				case 'D' :
                    //std::cout << " 2nd switch inside param D" << std::endl;
					Kd += dp[2];
					dp[2] *= 0.9;
					break;
				default:
                    std::cout << "Invalid Param" << std::endl;
				}
				improved = 1;
			}
		}
		switch(param) {
			case 'P' :
                //std::cout << "3rd switch Inside P" << std::endl;
				if(improved == 1) Kp += dp[0];
				else Kp -= 2*dp[0];
				break;
			case 'I' :
                //std::cout << "3rd switch Inside I" << std::endl;
				if(improved == 1) Ki += dp[1];
				else Ki -= 2*dp[1];
				break;
			case 'D' :
                  //std::cout << "3rd switch Inside D" << std::endl;
				if(improved == 1) Kd += dp[2];
				else Kd -= 2*dp[2];
				break;
			default:
                std::cout << "Invalid Param" << std::endl;
		}
	}
}




 




