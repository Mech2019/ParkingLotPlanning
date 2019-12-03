#ifndef A_STAR_H
#define A_STAR_H

#include <queue>
#include <math.h>
#include <vector>
#include <functional>
#include <unordered_set>

#include "state.h"
#include "map.h"
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

/*
 * This function calculates distance between a carstate and a state/ carstate
 */
template <class T>
double calc_state_distance(CarState *carstate, T* state){
	double car_x = carstate->get_x();
	double car_y = carstate->get_y();
	double state_x = state->get_x();
	double state_y = state->get_y();
	
	return (std::sqrt((car_x - state_x)*(car_x - state_x) 
		+ (car_y - state_y)*(car_y - state_y)));
}

/*
 * This is an A* search node
 */
class SearchNode{
private:
	double f,g,h;
	SearchNode *parent;
	std::vector<SearchNode *> child;
	CarState *self_state;
public:
	SearchNode(CarState *state);
	~SearchNode();

	double get_f();
	double get_g();
	double get_h();
	CarState *get_state();
	SearchNode *get_parent();

	void set_f();
	void set_g(double g);
	void set_h(double h);
	void set_parent(SearchNode *p);
	void insert_child(SearchNode *c);
};

// min_heap comparator
struct  SearchNodeHeapComparator {
	bool operator()(SearchNode *lhs, SearchNode *rhs){
		return lhs->get_f() > rhs->get_f();
	}
};

struct  SearchNodeComparator {
	bool operator()(SearchNode *lhs, SearchNode *rhs){
		auto lhs_state = lhs->get_state();
		auto rhs_state = rhs->get_state();
		isExactSameState(lhs_state, rhs_state);
	}
};


struct SearchNodeHasher{
	size_t operator()(SearchNode *node) const{
		return std::hash<double>{}(node->get_f());
	}
};

/*
 * This is an A* search class
 */
class A_star{
private:
	double goal_tol;
	CarState *start;
	CarState *goal;
	SearchNode *start_node;
	State* shop_location;
	std::vector<CarState *> path;
	std::unordered_set<State*, StateHasher, StateComparator> goal_list;
public:
	A_star(CarState *st, double tol);
	// ~A_star();
	int search(static_map *env);
	int backtrack(SearchNode *node);
	bool isGoal(CarState *cur);
	void update_goal_list(static_map *env);
	double calculate_heurisitcs(SearchNode *node);
	std::vector<CarState *> get_path();
	void free_search_tree();
};
// vector<SearchNode *> A_star(CarState *start, static_map *env);

#endif