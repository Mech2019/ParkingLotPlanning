#include "state.h"


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

void State::print_state() {
  cout << get_x() << ", " << get_y() << ", " << get_theta() << ", " <<
  get_flag() << endl;
}








