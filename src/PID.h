#pragma once
#include <iostream>

class PID
{
public:
	PID();
	~PID();

	void PID_init();
	// double PID_Cal(double targetValue, double *resultValue, double currValue, double* err, double *integral);
	double PID_Cal(double targetValue, double currValue);
	void SetPID(double _kp, double _ki, double _kd);

	double Kp = 0, Ki = 0, Kd = 0;

private:
	double err = 0;
	double err_next = 0;
	double err_last = 0;
	double integral = 0;         

};
