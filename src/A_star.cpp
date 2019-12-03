#include "A_star.h"
#include <cfloat>

using namespace std;

/* Search Node */
SearchNode::SearchNode(CarState *state){
	f = 0.0;
	g = DBL_MAX;
	h = 0.0;
	parent = NULL;
	self_state = state;
}

double SearchNode::get_f() {return f;}
double SearchNode::get_g() {return g;}
double SearchNode::get_h() {return h;}
CarState *SearchNode::get_state() {return self_state};
SearchNode *SearchNode::get_parent() {return parent;}

void SearchNode::set_f(double F) {this->f = this->g + this->h;}
void SearchNode::set_g(double G) {this->g = G;}
void SearchNode::set_h(double H) {this->h = H;}
void SearchNode::set_parent(SearchNode *p) {this->parent = p;}
void insert_child(SearchNode *c) {this->child.push_back(c);}

/* Search Class */
A_star::A_star(CarState *s, , State *vg, double tol){
	start = st;
	this->virtual_goal = vg;
	goal_tol = tol;
	this->start_node = new SearchNode(this->start);
}

int search(static_map *env){
	// motion primitive step cost
	static double step_cost = (car_speed * DURATION);
	// goal flag
	bool goal_reached = false;
	// initialize the node
	if (!this->start_node){
		this->start_node = new SearchNode(this->start);
	}
	start_node->set_g(0.0);
	start_node->set_h(calculate_heuristics(start_node));
	start_node->set_f();
	// initialize search queue
	priority_queue<SearchNode *, vector<SearchNode*>, SearchNodeHeapComparator> search_q;
	// initialize visited set
	unordered_set<SearchNode*, SearchNodeHasher, SearchNodeComparator> visited;
	// initialize a cost map
	unordered_map<SearchNode*, double, SearchNodeHasher, SearchNodeComparator> cost_map;
	cost_map[start_node] = start_node->get_g();
	// push the start node to start the search
	search_q.push(start_node);
	// goal for backtrack
	SearchNode *backtrack_node;

	// start searching
	while (!goal_reached && !search_q.empty()){
		auto tmp_node = search_q.top();
		search_q.pop();
		// check goal reaching
		if (isGoal(tmp_node->get_state())){
			backtrack_node = tmp->node;
			goal_reached = true;
			break;
		}
		// normal search routine
		if (visited.find(tmp_node) == visited.end()){
			visited.insert(tmp_node);
			// compute successor by using primitives
			vector<vector<CarState>> primitive_result;
			auto tmp_state = tmp_node->get_state();
			tmp_state->compute_primitive(primitive_result, env->get_occupied_slots());
			// update successor's g value and build the new node 
			vector<CarState> end_points;
			for (int i = 0; i < primitive_result.size(); i++){
				// only grap the end point of each primitive calculation
				end_points.push_back(primitive_result[i][primitive_result[i].size() - 1]);
			}
			// iterate over end_points for pq update
			for (int i = 0; i < end_points.size(); i++){
				auto primitive_state = &(end_points[i]);
				if (cost_map.find(primitive_state) == cost_map.end()){
					cost_map[primitive_state] = cost_map[tmp_state] + step_cost;
				}
				if (visited.find(primitive_state) == visited.end()){
					if (cost_map[primitive_state] > cost_map[tmp_state] + step_cost){
						cost_map[primitive_state] = cost_map[tmp_state] + step_cost;
						// make a new node and push into the pq
						SearchNode* new_node;
						new_node = new SearchNode(primitive_state);
						new_search_node->set_g(cost_map[tmp_state] + step_cost);
						new_search_node->set_h(calculate_heuristics(new_search_node));
						new_search_node->set_f();
						// set parent for backtracking
						new_search_node->set_parent(tmp_node);
						// push the node into the search queue
						search_q.push(new_search_node);
						// include the child in the parent node
						tmp_node->insert_child(new_node);
					}
				}
			}
		}
	}
	// no goal found
	if (!goal_reached){
		printf("No goal found.\n");
		free_search_tree();
		return -1;
	}
	// back tracking
	if (backtrack(backtrack_node) != 0){
		printf("Backtrack failed\n");
		free_search_tree();
		return -1;
	}
	// recycle the search tree
	free_search_tree();
	return 0;
}

int A_star::backtrack(SearchNode *node){
	auto tmp = node;
	while (tmp && tmp != this->start_node){
		this->path.insert(this->path.begin(), tmp->get_state());
		tmp = tmp->get_parent();
	}
	if (tmp != this->start_node){
		printf("start_node not found in backtrack\n");
		path.clear();
		return -1;
	}
	return 0;
}

bool A_star::isGoal(CarState *cur){
	for (auto it = goal_list.begin(); it != goal_list.end(); it++){
		if (calc_state_distance(cur, *it) <= goal_tol)
			return true;
	}
	return false;
}

void A_star::update_goal_list(static_map *env){
	auto env_goal_list = env->get_goal_list();
	for (auto it = env_goal_list.begin(); it != env_goal_list.end(); it++){
		this->goal_list.insert(*it);
	}
}

double A_star::calculate_heuristics(SearchNode *node){
	double result = DBL_MAX;
	auto env_goal_list = env->get_goal_list();
	for (auto it = env_goal_list.begin(); it != env_goal_list.end(); it++){
		result = MIN(result, calc_state_distance(node->get_state(), *it) 
								+ calc_state_distance(*it, virtual_goal));
	}
	return result;
}

std::vector<CarState *> A_star::get_path() {return this->path;}

void A_star::free_search_tree(){
	// use BFS to free the entire search tree
}

