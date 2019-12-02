#ifndef A_STAR_H
#define A_STAR_H

#include <queue>
#include <math.h>

#include "state.h"
#include "util.h"

/*
 * This function gives the ability to compare b/t a CarState and a State
 */
template <class T>
bool isExactSameState(CarState *carstate, T* state){
	if (carstate->get_x() != state->get_x() 
		|| carstate->get_y() != state->get_y()
		|| carstate->get_theta() != state->get_theta()) {
		return false;
	}
	return true;
}

template <class T>
double calc_state_distance(CarState *carstate, T* state){
	double car_x = carstate->get_x();
	double car_y = carstate->get_y();
	double state_x = state->get_x();
	double state_y = state->get_y();
	
	return (std::sqrt((car_x - state_x)*(car_x - state_x) 
		+ (car_y - state_y)*(car_y - state_y)));
}

class SearchNode{
private:
	double f,g,h;
	SearchNode *parent;
	CarState *self_state;
public:
	SearchNode(CarState *state);

	double get_f();
	double get_g();
	double get_h();
	SearchNode *get_parent();

	double set_f();
	double set_g();
	double set_h();
	void set_parent();
};

// min_heap comparator
struct  SearchNodeComparator {
	bool operator()(SearchNode *lhs, SearchNode *rhs){
		return lhs->get_f() > rhs->get_f();
	}
};

#endif