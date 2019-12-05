#include <iostream>
#include "state.h"
#include "util.h"

using namespace std;

State::State() {
	x = 0;
	y = 0;
	theta = 0;
	flag = 0;
}

State::State(double x_, double y_, double theta_, bool flag_) {
	x = x_;
	y = y_;
	theta = theta_;
	flag = flag_; // default forward
}

void State::update_state(int x_, int y_, double theta_, bool flag_) {
	x = x_;
	y = y_;
	theta = theta_;
	flag = flag_;
}

double State::get_x() {
	return x;
}

double State::get_y() {
	return y;
}

double State::get_theta() {
	return theta;
}

bool State::get_flag() {
	return flag;
}

void State::set_x(double x_) {
	x = x_;
}

void State::set_y(double y_) {
	y = y_;
}

void State::set_theta(double theta_) {
	theta = theta_;
}

bool State::operator==(State *rhs) const{
	if (this->x != rhs->get_x() || this->y != rhs->get_y()
		|| this->theta != rhs->get_theta() 
		|| this->flag != rhs->get_flag())
		return false;
	return true;
}

// =========================================

CarState::CarState() : State() {
	delta = 0;
}

CarState::CarState(double x_, double y_,
	double theta_, bool flag_, double delta_) : State(x_, y_, theta_, flag_) {
	delta = delta_;
}

double CarState::get_delta() {
	return delta;
}

void CarState::set_delta(double delta_) {
	delta = delta_;
}

ostream & operator<<(ostream & os, CarState & car)
{
	os << car.get_x() << ", " << car.get_y() << ", " << car.get_theta() << ", " << TODEG(car.get_delta());
	return os;
}


