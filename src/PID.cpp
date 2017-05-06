#include "PID.h"
#include <assert.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID()
: p_error_(0)
, i_error_(0)
, d_error_(0)
, Kp_(0)
, Ki_(0)
, Kd_(0)
, has_prev_cte_(false)
, prev_cte_(0)
{
}

PID::~PID()
{
}

void PID::Init(double Kp, double Ki, double Kd)
{
	Kp_ = Kp;
	Ki_ = Ki;
	Kd_ = Kd;

	p_error_ = 0;
	d_error_ = 0;

	prev_cte_ = 0;
	has_prev_cte_ = false;
	i_error_ = 0;
}

void PID::UpdateError(double cte)
{
	if (!has_prev_cte_) {
		prev_cte_ = cte;
		has_prev_cte_ = true;
	}

	p_error_ = cte;
	i_error_ += cte;
	d_error_ = cte - prev_cte_;

	prev_cte_ = cte;
}

double PID::TotalError()
{
	return -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
}

