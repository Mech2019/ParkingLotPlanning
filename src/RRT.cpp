#include "RRT.h"
#include <ctime>
#include <math.h>
#include <random>
using namespace std;

/* RRT search */
RRT::RRT(CarState *start_, CarState *goal_, double Vs, double Vg){
	this->start = new RRT_node(start_, Vs);
	this->goal = new RRT_node(goal_, Vg);
	q_end = NULL;
	tree.push_back(start);
	this->count = 0;
}

RRT_node *RRT::sample_new_node(){
	// std::default_random_engine generator;
	// std::uniform_int_distribution<int> distribution_node(0,tree.size()-1);
	// int seed = distribution_node(generator);
	// printf("tree_size = %d\n", this->tree.size());
	int seed = rand() % (this->tree.size());
	// printf("seed = %d\n", seed);
	return tree[seed];
}

RRT_node *RRT::generate_new_node(RRT_node *q_near, static_map *env){
	// vector<double> V = {-2, -1, 1, 2};
	// srand((unsigned) time(0));
	vector<double> d_delta = { TORAD(-30),TORAD(-25),TORAD(-20),TORAD(-15),TORAD(-10),
		TORAD(-5),TORAD(0),TORAD(5),TORAD(10),TORAD(15),TORAD(20),TORAD(25),TORAD(30)};
	// srand(time(NULL));
	// std::default_random_engine generator;
 //  	std::uniform_real_distribution<double> distribution_v(V_MIN, V_MAX);
 //  	std::uniform_int_distribution<int> distribution_del(0,d_delta.size()-1);

	double rand_vel = V_MIN + (double)rand() / RAND_MAX * (V_MAX - V_MIN);
	// double rand_vel = distribution_v(generator);
	// double rand_ddel = distribution_del(generator);
	

	double rand_ddel = dDEL_MIN + (double)rand() / RAND_MAX * (dDEL_MAX - dDEL_MIN);
	// double rand_ddel = d_delta[rand()%(d_delta.size())];
	// double rand_ddel = 0;

	rand_vel *= ((rand()%2) == 0? -1:1);

	// calculate vehicle trajectory by vehicle kinematics
	double dt = RRT_DURATION / RRT_SAMPLE;
	// initialization
	vector<RRT_node *> state_vec;
	state_vec.push_back(q_near);
	
	for (int i = 0; i < RRT_SAMPLE; i++) {
		auto prev_node = state_vec[i];
		auto prev_state = prev_node->get_state();
		double vel = prev_node->get_velocity();
		double dtheta = vel / wheel_base * rand_ddel;
		// update
		double x = prev_state->get_x() + vel * cos(prev_state->get_theta()) * dt;
		double y = prev_state->get_y() + vel * sin(prev_state->get_theta()) * dt;
		double theta = prev_state->get_theta() + dtheta * dt;
		double v = rand_vel;
		// build a new state
		CarState *new_car = new CarState(x, y, theta, vel>0, rand_ddel);
		RRT_node *new_node = new RRT_node(new_car, rand_vel);
		// collision check
		auto obstacles = env->get_occupied_slots();
		for (auto obs : obstacles){
			if (RRT_collision_check(obs, new_car))
				return NULL;
		}
		// printf("found a node\n");
		state_vec.push_back(new_node);
		// build the individual fine path
	}
	
	auto result = state_vec[state_vec.size() - 1];
	// push the last node into the tree
	result->set_velocity(rand_vel);
	result->set_parent(q_near);
	result->set_path(state_vec);
	q_near->insert_child(result);
	this->tree.push_back(result);
	// printf("tree_size = %d\n", this->tree.size());
	this->count += 1;
	return result;
}

void RRT::backtrack(RRT_node *q_end){
	this->path.clear();
	RRT_node *q_tmp = q_end;
	while (q_tmp){
		auto p = q_tmp->get_path();
		this->path.insert(this->path.begin(), p.begin(), p.end());
		q_tmp = q_tmp->get_parent();
	}
}

void RRT::search(static_map *env){
	while(count <= MAX_COUNT){
		if (count % 100 == 0 && count)
			printf("tree size %d\n", count);
		RRT_node *q_rand = sample_new_node();
		RRT_node *q_new = generate_new_node(q_rand, env);
		if (q_new){
			double d = q_new->calc_RRT_node_dist(this->goal);
			// if (d <= 4.0)
				// printf("distance = %lf, speed = %lf\n", d, q_new->get_velocity());
		 	// cout << this->goal->get_state() << endl;
		 	if (d <= TOLERANCE){
				printf("goal reached.\n");
				q_end = q_new;
				backtrack(q_end);
				return;
			}
		}
	}

	printf("goal not reached.\n");

	// dbg visualization
	printf("expanded state\n");
	ofstream traj_file("traj.csv");
	for (auto rrt_node : tree){
		auto p = rrt_node->get_state();
		traj_file << p << endl;
	}
	traj_file.close();
}

std::vector<CarState *> RRT::get_path(){return this->path;}

/* RRT node */
RRT_node::RRT_node(CarState *state_){
	this->state = state_;
	parent = NULL;
}
RRT_node::RRT_node(CarState *state_, double v_){
	this->state = state_;
	this->v = v_;
	parent = NULL;
}
// distance function
double RRT_node::calc_RRT_node_dist(RRT_node *rhs){
	double dx = state->get_x() - rhs->get_state()->get_x();
	double dy = state->get_y() - rhs->get_state()->get_y();
	double dtheta = min(state->get_theta() - rhs->get_state()->get_theta(),
		state->get_theta() - (rhs->get_state()->get_theta() + PI));
	return (sqrt(dx*dx + dy*dy + dtheta*dtheta));
	// return (sqrt(dx*dx + dy*dy));
}

// set functions
void RRT_node::set_velocity(double v_) {this->v = v_;}
void RRT_node::insert_child(RRT_node *child_) {this->child.push_back(child_);}
void RRT_node::set_parent(RRT_node *parent_) {this->parent = parent_;}

// get functions
CarState *RRT_node::get_state(){return this->state;}

double RRT_node::get_velocity(){return this->v;}

std::vector<RRT_node *> RRT_node::get_child(){return this->child;}

RRT_node *RRT_node::get_parent(){return this->parent;}

void RRT_node::set_path(std::vector<RRT_node *> state_vec){
	for (auto s:state_vec)
		path_from_parent.push_back(s->get_state());
}

std::vector<CarState *> RRT_node::get_path(){
	return path_from_parent;
}

/* RRT helper functions */
bool linesegmentcheck(double x1, double x2, double x3, double x4,
								double y1, double y2, double y3, double y4){
	if (x1 == x2 && x3 == x4){
		// printf("case 1\n");
		// printf("x1==x3? %d\n", x1==x3);
		if (x1 == x3){
			// printf("im here\n");
			// printf("y3: %lf \n", (y3-y1)*(y3-y2));
			// printf("y4: %lf \n", (y4-y1)*(y4-y2));
			if ((y3-y1)*(y3-y2) <= 0 || (y4-y1)*(y4-y2) <= 0){
				return true;
			}
		}
	} else if (x1 == x2 && x3 != x4) {
		// printf("case 2\n");
		double m2 = (y4 - y3) / (x4 - x3);
		double b2 = y3 - m2 * x3;

		double x_intersect = x1;
		double y_intersect = m2 * x_intersect + b2;
		if ((x_intersect-x1)*(x_intersect-x2) <= 0
			&& (x_intersect-x3)*(x_intersect-x4) <= 0
			&& (y_intersect-y1)*(y_intersect-y2) <= 0
			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
			return true;
		}
	} else if (x1 != x2 && x3 == x4) {
		// printf("case 3\n");
		double m1 = (y2 - y1) / (x2 - x1);
		double b1 = y1 - m1 * x1;

		double x_intersect = x3;
		double y_intersect = m1 * x_intersect + b1;
		if ((x_intersect-x1)*(x_intersect-x2) <= 0
			&& (x_intersect-x3)*(x_intersect-x4) <= 0
			&& (y_intersect-y1)*(y_intersect-y2) <= 0
			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
			return true;
		}
		// printf("case3, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
	} else {
		// calculate slope and y-intersection for line
		double m1 = (y2 - y1) / (x2 - x1);
		double b1 = y1 - m1 * x1;
		double m2 = (y4 - y3) / (x4 - x3);
		double b2 = y3 - m2 * x3;
		// printf("line 1: m1 = %lf, b1 = %lf\n", m1, b1);
		// printf("line 2: m2 = %lf, b2 = %lf\n", m2, b2);
		// calculate intersection coordinate
		double x_intersect = (b2-b1)/(m1-m2);
		double y_intersect = m1 * x_intersect + b1;
		// check if the intersection point is on both segment
		if ((x_intersect-x1)*(x_intersect-x2) <= 0
			&& (x_intersect-x3)*(x_intersect-x4) <= 0
			&& (y_intersect-y1)*(y_intersect-y2) <= 0
			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
			return true;
		}
		// printf("case4, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
	}
	// printf("im here 2\n");
	return false;
}