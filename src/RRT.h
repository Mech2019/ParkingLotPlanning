#ifndef RRT_H
#define RRT_H

#include <vector>

#include "state.h"
#include "map.h"
#include "util.h"

static const double TOLERANCE = 1;
static const int RRT_SAMPLE = 10;
static const int MAX_COUNT = 10000;
static const double RRT_DURATION = 0.5;
static const double BIAS = 0.10;
static const double A_MAX = 3;	// maximum acceleration of 3 m/s^2
static const double A_MIN = -3; // maximum deceleration
static const double dDEL_MAX = TORAD(30);	// maximum change of delta in a second
static const double dDEL_MIN = -TORAD(30); // minimum change of delta in a second

/* RRT search node and class */
class RRT_node{
private:
	CarState *state;
	RRT_node *parent;
	std::vector<RRT_node *> child;
	double v;
public:
	RRT_node(CarState *state_);
	RRT_node(CarState *state_, double v_);
	// distance function
	double calc_RRT_node_dist(RRT_node *rhs);

	// set functions
	void set_velocity(double v_);
	void insert_child(RRT_node *child_);
	void set_parent(RRT_node *parent_);

	// get functions
	CarState *get_state();
	double get_velocity();
	std::vector<RRT_node *> get_child();
	RRT_node *get_parent();
};

class RRT{
private:
	RRT_node *start;
	RRT_node *goal;
	RRT_node *q_end;
	std::vector<RRT_node *> tree;
	std::vector<CarState *> path;
	int count;
public:
	RRT(CarState *start_, CarState *goal_, double Vs, double Vg);
	RRT_node *sample_new_node();
	RRT_node *generate_new_node(RRT_node *q_near, static_map *env);
	void backtrack(RRT_node *q_end);
	void search(static_map *env);
	std::vector<CarState *> get_path();
};

/* RRT collision check helper functions */
// line segmentcheck for two line segments
bool linesegmentcheck(double x1, double x2, double x3, double x4,
						double y1, double y2, double y3, double y4);
// inflate state into a rectangle based on its heading angles
template <class T>
std::vector<std::vector<double>> inflate(T *s){
	double theta = s->get_theta();
	// ll, ul, ur, lr, ll
	vector<double> x_coord = {-car_len/2, -car_len/2, car_len/2,  car_len/2, -car_len/2};
	vector<double> y_coord = {-car_wid/2,  car_wid/2, car_wid/2, -car_wid/2, -car_wid/2};

	vector<vector<double>> result(2, vector<double>(5, 0.0));

	for (int i = 0; i < 5; i++){
		result[0][i] = cos(theta) * x_coord[i] - sin(theta) * y_coord[i] + s->get_x();
		result[1][i] = sin(theta) * x_coord[i] + cos(theta) * y_coord[i] + s->get_y();
	} 
	return result;
}
// RRT_collision_check main body
template <class T>
bool RRT_collision_check(State *s1, T *s2){
	// inflate state 1 and state 2
	vector<vector<double>> inflated_s1 = inflate(s1);
	vector<vector<double>> inflated_s2 = inflate(s2);
	// check line segment
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			double x1 = inflated_s1[0][i], x2 = inflated_s1[0][i+1];
			double y1 = inflated_s1[1][i], y2 = inflated_s1[1][i+1];

			double x3 = inflated_s2[0][j], x4 = inflated_s2[0][j+1];
			double y3 = inflated_s2[1][j], y4 = inflated_s2[1][j+1];

			// printf("%lf, %lf, %lf, %lf\n", x1,y1,x2,y2);
			// printf("%lf, %lf, %lf, %lf\n", x3,y3,x4,y4);
			// printf("result = %d\n", linesegmentcheck(x1,x2,x3,x4,y1,y2,y3,y4));
			if (linesegmentcheck(x1,x2,x3,x4,y1,y2,y3,y4) == true)
				return true;
		}
	}
	// out of bound check
	// if (s1->get_x() < 0 || s1->get_x() > map_wid)
	// 	return true;
	if (s2->get_x() < 0 || s2->get_x() > map_wid)
		return true;
	// if (s1->get_y() < 0 || s1->get_y() > map_len)
	// 	return true;
	if (s2->get_y() < 0 || s2->get_y() > map_len)
		return true;
	return false;
}


#endif