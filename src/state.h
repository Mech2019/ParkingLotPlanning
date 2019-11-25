#ifndef STATE_H
#define STATE_H

#include <vector>
#include <iostream>

using namespace std;

class State{
private:
	double x;
	double y;
	double theta;
	bool flag; //ego_vehicle: forward/backward; map: empty/full

public:
	//constructor
	State();
	State(double x_, double y_, double theta_, bool flag_);

	void update_state(int x_, int y_, double theta_, bool flag_);
	double get_x();
	double get_y();
	double get_theta();
	bool get_flag();
	void set_x(double x_);
	void set_y(double y_);
	void set_theta(double theta_);
};


class CarState : public State {
private:
	double delta; // steering angle
	CarState nextCarState(CarState car, double dt) const;

public:
	CarState();
	CarState(double x_, double y_, double theta_, bool flag_, double delta_);

	friend ostream& operator<<(ostream& os, CarState& car);

	double get_delta();
	void set_delta(double delta_);

	void compute_primitive(std::vector<std::vector<CarState> > &result) const;
};

#endif