#include "planner.h"
#include <chrono>
#include <string>

using namespace std;

static const char *traj_fn = "traj.csv";
static const char *map_fn = "slot";
static const char *csv_fn = ".csv";


int main(){
	srand((unsigned) time(0));

	/* test map parsing */
	vector<vector<double>> input;
	parse_static_map(map_name, input);
	static_map *env = new static_map(map_wid, map_len, input);
	if (!env){
		printf("Dynamic allocation for environment failed\n");
		return -1;
	}
	// print to see map info
	printf("map size is %lfx%lf, total number of slots are %d\n", 
		env->get_map_width(), env->get_map_length(), env->get_slot_num());


	/* A_star replanning */
	CarState *ego_vehicle;
	State *virtual_goal = new State(50.0, 30.0, 0.0, 0.0);
	// State *virtual_goal = new State(80.0, 50.0, 0.0, 0.0);
	A_star *a_star; 

	vector<CarState *> final_path;
	vector<CarState *> path;


	// initialize
	ego_vehicle = new CarState(8.5, 2.75, PI/2, 1, 0.0);
	a_star = new A_star(ego_vehicle, virtual_goal, 2.0);

	SearchNode *goal = NULL;
	CarState *prev_state = NULL;
	CarState *curr_state = NULL;
	double term_tol = 0.001;
	int i = 0;
	while (true){
		string fn = map_fn + to_string(i) + csv_fn;
		if (prev_state && curr_state){
			double dx = curr_state->get_x() - prev_state->get_x();
			double dy = curr_state->get_y() - prev_state->get_y();
			double dtheta = curr_state->get_theta() - prev_state->get_theta();
			if (sqrt(dx*dx+dy*dy+dtheta*dtheta) <= term_tol)
				break;
		}
		auto start_time = chrono::steady_clock::now();
		printf("i = %d\n", i++);
		env->update_goal_list(ego_vehicle, fn);
		a_star->update_goal_list(env);
		// printf("finished updating goal.\n");
		if (a_star->search(env) == 0){
			// printf("finished searching\n");
			path = a_star->get_path();
			// printf("A* path size %d\n", path.size());
			for (auto p : path){
				final_path.push_back(new CarState(p->get_x(), p->get_y(), 
					p->get_theta(), p->get_flag(), p->get_delta()));
			}
		}
		prev_state = curr_state;
		if (path.size()){
			curr_state = path[path.size()-1];
		} 
		goal = a_star->get_goal();
		cout << "goal: " << goal->get_state() << endl;
		cout << "curr " << curr_state << endl;

		ego_vehicle->set_x(final_path[final_path.size() - 1]->get_x());
		ego_vehicle->set_y(final_path[final_path.size() - 1]->get_y());
		ego_vehicle->set_theta(final_path[final_path.size() - 1]->get_theta());
		ego_vehicle->set_delta(final_path[final_path.size() - 1]->get_delta());	
		// printf("finsied reinitialize ego_vehicle\n");
		a_star->set_start(ego_vehicle);
		auto end_time = chrono::steady_clock::now();
		cout << "seconds elapsed: " 
			<< (chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count())/1000.0 << endl;
	}
	// printf("Im here\n");


	/* RRT local planning */
	int n = 10;
	SearchNode *astar_goal = a_star->get_goal();
	CarState *RRT_start;
	if (final_path.size() > n)
		RRT_start = final_path[final_path.size()-n];
	else
		RRT_start = final_path[final_path.size() - 1];
	// printf("Im here 2\n");
	CarState *RRT_goal = new CarState();
	double dist = DBL_MAX;
	for (auto slot : env->get_slots()){
		if (!slot->get_flag()){
			// double d = calc_state_distance(astar_goal->get_state(), slot);
			double d = calc_state_distance(final_path[final_path.size() - 1], slot);
			cout << "map slot:" << slot << endl;
			cout << "dist: " << d <<endl;
			if (d < dist){
				dist = d;
				RRT_goal->set_x(slot->get_x());
				RRT_goal->set_y(slot->get_y());
				RRT_goal->set_theta(slot->get_theta());
				RRT_goal->set_vel(0.0);
			}
		}
	}
	cout << "RRT start:" << RRT_start << endl;
	cout << "RRT goal:" << RRT_goal << endl;

	RRT *rrt = new RRT(RRT_start, RRT_goal, 2.0, 0.0);
	printf("start rrt searching.\n");

	auto start_time = chrono::steady_clock::now();

	rrt->search(env);

	auto end_time = chrono::steady_clock::now();
    cout << "seconds elapsed: " 
        << (chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count())/1000.0 << endl;
	
	auto final_rrt_path = rrt->get_path();
	if (final_rrt_path.size() > n){
		final_path.insert(final_path.end()-n, final_rrt_path.begin(), final_rrt_path.end());
		final_path.erase(final_path.end()-n, final_path.end());
	} else {
		final_path.insert(final_path.end(), final_rrt_path.begin(), final_rrt_path.end());
	}


	/* write for visualization */
	if (final_path.size() > 0){
		printf("finished loading path\n");
		ofstream traj_file(traj_fn);
		for (auto p:final_path){
			traj_file << p->get_x() << "," << p->get_y() << "," << p->get_theta() << endl;
		}
		traj_file.close();
		printf("finished writing\n");
	}
	a_star->free_search_tree();

	delete env;
	delete goal;
	delete ego_vehicle;
	delete a_star;
	delete rrt;
}