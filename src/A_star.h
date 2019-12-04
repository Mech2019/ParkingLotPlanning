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

static const int max_expansion = 75;
// /*
//  * This function gives the ability to compare b/t a CarState and a State
//  */
// template <class T>
// bool isExactSameState(CarState *carstate, T* state){
// 	if (carstate->get_x() != state->get_x() 
// 		|| carstate->get_y() != state->get_y()
// 		|| carstate->get_theta() != state->get_theta()) {
// 		return false;
// 	}
// 	return true;
// }

/*
 * This function calculates distance between a carstate and a state/ carstate
 */
template <class T>
double calc_state_distance(T *carstate, State* state){
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
	std::vector<CarState *> path_from_parent; 
	CarState *self_state;

public:
	SearchNode(CarState *state);
	// ~SearchNode();

	double get_f();
	double get_g();
	double get_h();
	CarState *get_state();
	SearchNode *get_parent();
	std::vector<SearchNode *> get_child();
	std::vector<CarState *> get_path();

	void set_f();
	void set_g(double g);
	void set_h(double h);
	void set_path(std::vector<CarState> primitive_path);
	void set_parent(SearchNode *p);
	void insert_child(SearchNode *c);

	void free_self_state();

	bool cmp(CarState *rhs, double tol);
};

// min_heap comparator
struct  SearchNodeHeapComparator {
	bool operator()(SearchNode *lhs, SearchNode *rhs) const{
		return lhs->get_f() > rhs->get_f();
	}
};

struct  SearchNodeComparator {
	bool operator()(SearchNode *lhs, SearchNode *rhs) const{
		auto lhs_state = lhs->get_state();
		auto rhs_state = rhs->get_state();
		if (lhs_state->get_x() != rhs_state->get_x() 
		|| lhs_state->get_y() != rhs_state->get_y()) {
			return false;
		}
		return true;
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
	SearchNode *goal;
	SearchNode *start_node;
	State* virtual_goal;
	std::vector<CarState *> path;
	std::unordered_set<State*, StateHasher, StateComparator> goal_list;
public:
	A_star(CarState *st, State *vg, double tol);
	void set_start(CarState *st);
	int search(static_map *env);
	int backtrack(SearchNode *node);
	bool isGoal(CarState *cur);
	void update_goal_list(static_map *env);
	double calculate_heuristics(SearchNode *node);
	std::vector<CarState *> get_path();
	SearchNode *get_goal();
	void free_search_tree();
};
// vector<SearchNode *> A_star(CarState *start, static_map *env);

#endif