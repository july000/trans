#include "PID.h"



PID::PID()
{
	PID_init();
}


PID::~PID()
{
}

void PID::PID_init()
{
	err = 0;
	err_last = 0;
	err_next = 0;
	integral = 0;
}

double PID::PID_Cal(double targetValue, double currValue)
{
	double calcResult;

	err = targetValue - currValue;
	integral += err;
	double max_integral=100;
	if(integral>max_integral){
		integral=max_integral;
	}else if(integral<-max_integral){
		integral=-max_integral;
	}
	calcResult = Kp*err + Ki*integral + Kd*(err - err_last);
	err_last = err;

	/*std::cout
		<< err << "	"
		<< integral << "	";*/

	return calcResult;
}

void PID::SetPID(double _kp, double _ki, double _kd)
{
	Kp = _kp;
	Ki = _ki;
	Kd = _kd;
}
