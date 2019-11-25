#include <iostream>
#include <vector>
#include "state.h"

using namespace std;

// compute next car state given current car state, speed (constant global)
// and steering angle after a time duration dt
CarState CarState::nextCarState(CarState car, double dt) const {
	return car;
}

// input: 
//	empty 2D vector of CarState to store sampled locations
//	Each row in this vector represents the trajectoy of 1 primitive
//	with the last element to be the final location after performing that primitive
// Output: filled 2D vector
void CarState::compute_primitive(vector<vector<CarState>> &result) const {
	double DURATION = 0.5; // time to drive
	int SAMPLE_POINTS = 10; // sampled points per each primitive
	double delta_MAX = 30.0 / 180.0*PI; // max steering angle to right
	double delta_MIN = -30.0 / 180.0*PI; // max steering angle to left
	vector<double> d_delta = {-15,-10,-5,0,5,10,15}; // change of steering angle for different primitive

	result.clear();
	for (double d : d_delta) {
		double new_delta = delta + d;
		if (new_delta < delta_MIN || new_delta > delta_MAX) continue;

		result.push_back(vector<CarState>());
		
		CarState new_car = *this;
		new_car.set_delta(new_delta);

		double dt = DURATION / SAMPLE_POINTS;
		for (int i = 0; i < SAMPLE_POINTS; i++) {
			result[result.size()-1].push_back(this->nextCarState(new_car,dt));
		}
	}
}

// Test
int main() {
	CarState car;
	int temp;
	car.set_delta(3.1415926);
	cout << car << endl;

	vector<vector<CarState>> result;
	car.compute_primitive(result);

	cin >> temp;

	return 0;
}