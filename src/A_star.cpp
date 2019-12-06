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
CarState *SearchNode::get_state() {return self_state;}
SearchNode *SearchNode::get_parent() {return parent;}
std::vector<SearchNode *> SearchNode::get_child() {return child;}
std::vector<CarState *> SearchNode::get_path(){return path_from_parent;}

void SearchNode::set_f() {this->f = this->g + this->h;}
void SearchNode::set_g(double G) {this->g = G;}
void SearchNode::set_h(double H) {this->h = H;}
void SearchNode::set_parent(SearchNode *p) {this->parent = p;}
void SearchNode::insert_child(SearchNode *c) {this->child.push_back(c);}
void SearchNode::set_path(std::vector<CarState> primitive_path){
	for (auto p:primitive_path){
		this->path_from_parent.push_back(new CarState(p.get_x(), p.get_y(), p.get_theta(), 0, 0));
	}
}
void SearchNode::free_self_state(){
	delete this->self_state;
}


bool SearchNode::cmp(CarState *rhs, double tol){
	double dx = this->get_state()->get_x() - rhs->get_x();
	double dy = this->get_state()->get_y() - rhs->get_y();
	double dtheta = this->get_state()->get_theta() - rhs->get_theta();

	if (sqrt(dx*dx+dy*dy+dtheta*dtheta) <= tol)
		return true;
	return false;
}
/* Search Class */
A_star::A_star(CarState *st, State *vg, double tol){
	start = st;
	this->virtual_goal = vg;
	goal_tol = tol;
	this->start_node = new SearchNode(this->start);
}
void A_star::set_start(CarState *st){
	this->start = st;
}
int A_star::search(static_map *env){
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
	// unordered_map<SearchNode*, double, SearchNodeHasher, SearchNodeComparator> cost_map;
	// cost_map[start_node] = start_node->get_g();
	// push the start node to start the search
	search_q.push(start_node);
	// // goal for backtrack
	// SearchNode *backtrack_node;
	//
	int expansion_cnt = 0;
	// start searching
	while (!goal_reached && !search_q.empty() && expansion_cnt < max_expansion){
		auto tmp_node = search_q.top();
		search_q.pop();
		// backtrack_node = tmp_node;
		// check goal reaching
		if (isGoal(tmp_node->get_state())){
			// backtrack_node = tmp_node;
			this->goal = tmp_node;
			 printf("target is at %lf, %lf\n", tmp_node->get_state()->get_x(), 
				tmp_node->get_state()->get_y());
			goal_reached = true;
			break;
		}
		// normal search routine
		if (visited.find(tmp_node) == visited.end()){
			expansion_cnt += 1;
			// printf("size %d, expanded %d\n", search_q.size(), expansion_cnt);
			// printf("currently at %lf, %lf, with f_value %lf\n", tmp_node->get_state()->get_x(), 
			// 	tmp_node->get_state()->get_y(), tmp_node->get_f());
			visited.insert(tmp_node);
			// compute successor by using primitives
			vector<vector<CarState>> primitive_result;
			auto tmp_state = tmp_node->get_state();
			tmp_state->compute_primitive(primitive_result, env->get_occupied_slots());
			// tmp_state->compute_primitive(primitive_result, env->get_slots());
			// printf("feasible primitive: %d\n", primitive_result.size());
			// update successor's g value and build the new node 
			vector<CarState> end_points;
			for (int i = 0; i < primitive_result.size(); i++){
				// only grab the end point of each primitive calculation
				end_points.push_back(primitive_result[i][primitive_result[i].size() - 1]);
				// printf("end_points %lf, %lf\n", end_points[i].get_x(), end_points[i].get_y());
			}
			// iterate over end_points for pq update
			for (int i = 0; i < end_points.size(); i++){
				// auto primitive_state = &(end_points[i]);
				auto primitive_state = new CarState(end_points[i].get_x(), end_points[i].get_y(),
					end_points[i].get_theta(),end_points[i].get_flag(),end_points[i].get_delta());
				// if (cost_map.find(primitive_state) == cost_map.end()){
				// 	cost_map[primitive_state] = cost_map[tmp_state] + step_cost;
				// }
				SearchNode* new_search_node;
				new_search_node = new SearchNode(primitive_state);
				if (visited.find(new_search_node) == visited.end()){
					// if (cost_map[primitive_state] > cost_map[tmp_state] + step_cost){
					// 	cost_map[primitive_state] = cost_map[tmp_state] + step_cost;
					// make a new node and push into the pq
					// new_search_node->set_g(cost_map[tmp_state] + step_cost);
					double theta = new_search_node->get_state()->get_theta();
					new_search_node->set_g(tmp_node->get_g() + step_cost + theta * theta);
					new_search_node->set_h(calculate_heuristics(new_search_node));
					new_search_node->set_f();
					// set parent for backtracking
					new_search_node->set_parent(tmp_node);
					new_search_node->set_path(primitive_result[i]);
					// printf("pushed: %lf, %lf\n", new_search_node->get_state()->get_x(), 
					// 	new_search_node->get_state()->get_y());
					// push the node into the search queue
					search_q.push(new_search_node);
					// include the child in the parent node
					tmp_node->insert_child(new_search_node);
					// }
				}
			}
		}
	}
	// no goal found
	if (!goal_reached && expansion_cnt < max_expansion){
		printf("No goal found.\n");
		free_search_tree();
		return -1;
	}
	if (expansion_cnt == max_expansion){
		this->goal = search_q.top();
	}
	// back tracking
	if (backtrack(this->goal->get_parent()) != 0){
		printf("Backtrack failed\n");
		free_search_tree();
		return -1;
	}

	return 0;
}

int A_star::backtrack(SearchNode *node){
	this->path.clear();
	auto tmp = node;
	while (tmp && tmp!=this->start_node){
		auto path_from_parent = tmp->get_path();
		// only preserve the first waypoint
		this->path.clear();
		// printf("num in path %d\n", path_from_parent.size());
		this->path.insert(this->path.begin(), path_from_parent.begin(), 
			path_from_parent.end());
		// pretmp = tmp;
		tmp = tmp->get_parent();
	}
	if (tmp != this->start_node){
		printf("start_node not found in backtrack\n");
		path.clear();
		return -1;
	}
	// printf("path size is %d\n", this->path.size());
	// this->path.insert(this->path.begin(), this->start_node->get_state());
	return 0;
}

bool A_star::isGoal(CarState *cur){
	for (auto it = goal_list.begin(); it != goal_list.end(); it++){
		if (calc_state_distance(cur, *it) <= goal_tol){
			// printf("currently at %lf, %lf\n", (cur)->get_x(), (cur)->get_y());
			// printf("goal is %lf, %lf\n", (*it)->get_x(), (*it)->get_y());
			// printf("distance is %lf\n", calc_state_distance(cur, *it));
			return true;
		}
	}
	return false;
}

// need to call this before the search!!!
void A_star::update_goal_list(static_map *env){
	auto env_goal_list = env->get_goal_list();
	this->goal_list.clear();
	for (auto it = env_goal_list.begin(); it != env_goal_list.end(); it++){
		this->goal_list.insert(*it);
	}
	printf("total empty goal list size %d\n", goal_list.size());
}

double A_star::calculate_heuristics(SearchNode *node){
	double result = DBL_MAX;
	State *closest;
	for (auto it = this->goal_list.begin(); it != this->goal_list.end(); it++){
		double theta = (node->get_state()->get_theta()); 
		double c = calc_state_distance(node->get_state(), *it) +
					 calc_state_distance(*it, virtual_goal);
		// result = goal_virtual_w * MIN(result, c);
		if (c < result)
			closest = *it;
		result = goal_virtual_w * MIN(result, c);
	}
	// printf("the nearest spot is:\n");
	// cout << node->get_state() << endl;
	// cout << closest << endl;
	// printf("heursitics = %lf\n", result);
	return result;
}

std::vector<CarState *> A_star::get_path() {return this->path;}

SearchNode *A_star::get_goal(){return this->goal;}

void A_star::free_search_tree(){
	// use BFS to free the entire search tree
	queue<SearchNode*> q;
	q.push(this->start_node);
	while (!q.empty()){
		auto tmp = q.front();
		q.pop();
		for (auto c : tmp->get_child()){
			q.push(c);
		}
		tmp->free_self_state();
		delete tmp;
	}
}

