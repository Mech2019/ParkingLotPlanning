#include "RRT.h"
#include <time.h>
#include <math.h>
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
	srand(time(NULL));
	int seed = rand()%(tree.size());
	return tree[seed];
}

RRT_node *RRT::generate_new_node(RRT_node *q_near, static_map *env){
	vector<double> V = {-2, -1, 1, 2};
	// srand(time(NULL));
	double rand_vel = V[rand()%3];
	double rand_ddel = dDEL_MIN + (double)rand() / RAND_MAX * (dDEL_MAX - dDEL_MIN);

	// calculate vehicle trajectory by vehicle kinematics
	double dt = RRT_DURATION / RRT_SAMPLE;
	// initialization
	auto q_tmp = q_near;
	auto tmp_state = q_tmp->get_state();
	double vel = q_tmp->get_velocity();
	double new_delta = tmp_state->get_delta() + rand_ddel;
	if (new_delta < delta_MIN || new_delta > delta_MAX) 
		return NULL;
	
	CarState new_car = *tmp_state;
	new_car.set_delta(new_delta);

	for (int i = 0; i < RRT_SAMPLE; i++) {
		// new_car = tmp_state->nextCarState(new_car, vel, dt);

		// calculate next state with different velocity
		double start_x = new_car.get_x();
		double start_y = new_car.get_y();
		double theta = new_car.get_theta();
		double delta = new_car.get_delta();
		double flag = new_car.get_flag();

		double curve_length = vel * dt;
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

		new_car = *(new CarState(new_x_global, new_y_global, new_theta, flag, delta));


		auto obstacles = env->get_occupied_slots();
		for (auto obs : obstacles){
			if (RRT_collision_check(obs, &new_car))
				return NULL;
		}
		tmp_state = &new_car;
	}
	auto end = new_car;
	auto result_state = new CarState(end.get_x(),end.get_y(), end.get_theta(), end.get_flag(), end.get_delta());
	cout << result_state << ", " << rand_vel << "," << TODEG(rand_ddel) << endl;
	RRT_node *result = new RRT_node(result_state);
	result->set_velocity(rand_vel);
	result->set_parent(q_near);
	q_near->insert_child(result);
	this->tree.push_back(result);
	this->count += 1;
	return result;
}

void RRT::backtrack(RRT_node *q_end){
	this->path.clear();
	RRT_node *q_tmp = q_end;
	while (q_tmp){
		this->path.insert(this->path.begin(), q_tmp->get_state());
		q_tmp = q_tmp->get_parent();
	}
}

void RRT::search(static_map *env){
	while(count <= MAX_COUNT){
		if (count % 100 == 0)
			printf("tree size %d\n", count);
		RRT_node *q_rand = sample_new_node();
		RRT_node *q_new = generate_new_node(q_rand, env);
		if (q_new){
			printf("distance = %lf\n", q_new->calc_RRT_node_dist(this->goal));
		 	// cout << this->goal->get_state() << endl;
		 	if (q_new->calc_RRT_node_dist(this->goal) <= TOLERANCE){
				printf("goal reached.\n");
				q_end = q_new;
				backtrack(q_end);
				return;
			}
		}
	}
	printf("goal not reached.\n");
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
	double dtheta = state->get_theta() - rhs->get_state()->get_theta();
	return (sqrt(dx*dx + dy*dy + dtheta*dtheta));
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