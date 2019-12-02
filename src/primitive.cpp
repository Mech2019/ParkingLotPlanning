#include <iostream>
#include <vector>
#include "state.h"
#include "util.h"

using namespace std;

// Declear usefull function header
bool total_collision_check(State *car, vector<State*> & obstacles);

// compute next car state given current car state, speed (constant global)
// and steering angle after a time duration dt
CarState CarState::nextCarState(CarState car, double dt) const {
	return car;
}

// input: 
//	empty 2D vector of CarState to store sampled locations
//	vector of obstacles to check for collision
// Output: 
//	filled 2D vector
//	Each row in this vector represents the trajectoy of 1 primitive
//	with the last element to be the final location after performing that primitive
<<<<<<< HEAD
void CarState::compute_primitive(vector<vector<CarState>> &result) const {
	double DURATION = 0.5; 	// time to drive
	int SAMPLE_POINTS = 10; // sampled points per each primitive
	double delta_MAX = 30.0 / 180.0 * PI; 	// max steering angle to right
	double delta_MIN = -30.0 / 180.0 * PI; 	// max steering angle to left
	// change of steering angle for different primitive
	vector<double> d_delta = {-15, -10, -5, 0, 5, 10, 15}; // change to radian!!!
=======
void CarState::compute_primitive(vector<vector<CarState>> &result, vector<State*> & obstacles) const {
	double DURATION = 0.5; // time to drive
	int SAMPLE_POINTS = 10; // sampled points per each primitive
	double delta_MAX = TORAD(30); // max steering angle to right
	double delta_MIN = TORAD(-30); // max steering angle to left
	vector<double> d_delta = {TORAD(-15),TORAD(-10),TORAD(-5),TORAD(0),TORAD(5),TORAD(10),TORAD(15)}; // change of steering angle for different primitive
>>>>>>> 2fee7c316ffd8b1149ee69e23e89969183023d99

	result.clear();
	for (double d : d_delta) {
		double new_delta = delta + d;
		if (new_delta < delta_MIN || new_delta > delta_MAX) continue;
		
		vector<CarState> temp;
		CarState new_car = *this;
		new_car.set_delta(new_delta);
		bool obstacle_free = true;
		double dt = DURATION / SAMPLE_POINTS;

		for (int i = 0; i < SAMPLE_POINTS; i++) {
			new_car = this->nextCarState(new_car, dt);
			temp.push_back(new_car);
			if (total_collision_check(&new_car, obstacles)) {
				obstacle_free = false;
				break;
			}
		}

		if (obstacle_free) result.push_back(temp);
	}
}

// Test
// int main() {
// 	CarState car;
// 	int temp;
// 	car.set_delta(3.1415926);
// 	cout << car << endl;

// 	vector<vector<CarState>> result;
// 	vector<State*> obstacle;
// 	car.compute_primitive(result, obstacle);

// 	cin >> temp;

// 	return 0;
// }