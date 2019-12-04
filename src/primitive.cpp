#include <iostream>
#include <vector>
#include "state.h"
#include "util.h"

using namespace std;

// compute next car state given current car state, speed (constant global)
// and steering angle after a time duration dt
CarState CarState::nextCarState(CarState start, double dt) const {
	double start_x = start.get_x();
	double start_y = start.get_y();
	double theta = start.get_theta();
	double delta = start.get_delta();
	double flag = start.get_flag();

	double curve_length = car_speed * dt;
	double turning_radius = abs(wheel_base / tan(delta)) + car_wid * 0.5;
	double dtheta = abs(curve_length / turning_radius);

	//if wheel turn right, reverse the change in heading
	if (delta < 0.0) {
		dtheta = -dtheta;
	}

	double new_theta = 0.0;
	new_theta = theta + dtheta;
	if (new_theta > 2.0 * PI) {
		new_theta -= 2.0 * PI;
	}
	else if (new_theta < 0.0) {
		new_theta += 2.0 * PI;
	}

	double new_x_global, new_y_global;
	double rot_x, rot_y, rot_theta;

	if (delta == 0.0) {
		new_x_global = start_x + cos(theta)*curve_length;
		new_y_global = start_y + sin(theta)*curve_length;
	}
	else {
		//find rotation center
		if (delta > 0.0) {
			rot_theta = theta + PI / 2.0;
		}
		else {
			rot_theta = theta - PI / 2.0;
		}

		rot_x = start_x + turning_radius * cos(rot_theta);
		rot_y = start_y + turning_radius * sin(rot_theta);
		double local_theta = atan2(start_y - rot_y, start_x - rot_x);

		//homogeneous tranformation to get new global coordinates
		new_x_global = cos(dtheta) * turning_radius * cos(local_theta)
			- sin(dtheta) * turning_radius * sin(local_theta)
			+ rot_x;
		new_y_global = sin(dtheta) * turning_radius * cos(local_theta)
			+ cos(dtheta) * turning_radius * sin(local_theta)
			+ rot_y;
	}

	CarState new_state(new_x_global, new_y_global, new_theta, flag, delta);

	return new_state;
}

// input: 
//	empty 2D vector of CarState to store sampled locations
//	vector of obstacles to check for collision
// Output: 
//	filled 2D vector
//	Each row in this vector represents the trajectoy of 1 primitive
//	with the last element to be the final location after performing that primitive
void CarState::compute_primitive(vector<vector<CarState>> &result, vector<State*> & obstacles) const {
	const double DURATION = 1.0; 	// time to drive
	const int SAMPLE_POINTS = 20; 	// sampled points per each primitive
	const double delta_MAX = TORAD(30); // max steering angle to right
	const double delta_MIN = TORAD(-30); // max steering angle to left
	vector<double> d_delta = { TORAD(-30),TORAD(-25),TORAD(-20),TORAD(-15),TORAD(-10),
		TORAD(-5),TORAD(0),TORAD(5),TORAD(10),TORAD(15),TORAD(20),TORAD(25),TORAD(30)}; // change of steering angle for different primitive

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
			if (total_collision_check(obstacles, &new_car)) {
				obstacle_free = false;
				break;
			}
		}
		// printf("obstacle free? %d\n", obstacle_free);
		if (obstacle_free) result.push_back(temp);
	}
}

// Test
//int main() {
//	CarState car;
//	int temp;
//	car.set_x(1.0);
//	car.set_y(1.0);
//	car.set_theta(TORAD(45));
//	car.set_delta(0);
//	cout << car << endl;
//
//	vector<vector<CarState>> result;
//	vector<State*> obstacle;
//	car.compute_primitive(result, obstacle);
//
//	for (auto x : result) {
//		for (auto y : x) {
//			cout << y.get_x() << "," << y.get_y() << "," << y.get_theta() << endl;
//		}
//		cout << "\n";
//	}
//
//	cin >> temp;
//	return 0;
//}