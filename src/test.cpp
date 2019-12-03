#include "planner.h"

using namespace std;

static const char *traj_fn = "traj.csv";

int main(){
	printf("This is a test script for functionality.\n");
	/* test map parsing */
	vector<vector<double>> input;
	parse_static_map(map_name, input);
	static_map *env = new static_map(map_wid, map_len, input);
	if (!env){
		printf("Dynamic allocation for environment failed\n");
		return -1;
	}
	// print to see map info
	printf("map size is %lfx%lf, total number of spots are %d\n", 
		env->get_map_width(), env->get_map_length(), env->get_slot_num());
	

	// /* test collision checking */
	// //  printf("Start check collision !\n");
	// State *state1 = new State(0, 0, 0, 0);
	// State *state2 = new State(10, 0, 0, 0);
	// CarState *carstate = new CarState(8.5, 2.75, PI/2, 1, 0.0);
	// cout << collision_check(carstate, state2) << endl;
	// cout << collision_check(carstate, carstate) << endl;

	// State *state3 = new State(0, 0, 0, 0);
	// State *state4 = new State(5, 3, 0.54, 0);
	// cout << collision_check(carstate, state4) << endl;

	// delete(state1);
	// delete(state2);
	// delete(carstate);
	// delete(state3);
	// delete(state4);
	// /* state comparator test */
	// State *s1 = new State(1.0, 1.0, 0.0, true);
	// State *s2 = new State(1.0, 1.0, 0.0, true);
	// printf("comparator test %d\n", (*s1 == *s2));

	// /* test sensor reading */
	// printf("printing reference:\n");
	// auto ref = env->get_slots();
	// for (auto a : ref){
	// 	cout << a << endl;
	// }

	// CarState *ego_vehicle = new CarState(8.5, 2.75, PI/2, 1, 0.0);
	// ofstream traj_file(traj_fn);
	// for (int i = 0; i < 20; i++){
	// 	traj_file << ego_vehicle->get_x() << "," << ego_vehicle->get_y() << "," << ego_vehicle->get_theta() << endl;
	// 	ego_vehicle->set_y(ego_vehicle->get_y() + 0.1*5.0);
	// 	printf("line of sight test: \n");
	// 	printf("ego_vechicle state:\n");
	// 	cout << ego_vehicle << endl;
	// 	env->update_goal_list(ego_vehicle);
	// 	auto goal = env->get_goal_list();
	// 	printf("result:\n");
	// 	for (auto it = goal.begin(); it != goal.end(); it++){
	// 		auto slot = *it;
	// 		cout << slot << endl;
	// 	}
	// }
	// traj_file.close();
	// delete ego_vehicle;
	// delete env;

	/* test A_star */
	CarState *ego_vehicle = new CarState(8.5, 2.75, PI/2, 1, 0.0);
	State *virtual_goal = new State(50.0, 30.0, 0.0, 0.0);
	A_star *a_star = new A_star(ego_vehicle, virtual_goal, 2.0);
	vector<CarState *> path;

	env->update_goal_list(ego_vehicle);
	a_star->update_goal_list(env);
	if (a_star->search(env) == 0){
		printf("finished searching\n");
		path = a_star->get_path();
		if (path.size() > 0){
			printf("finished loading path\n");
			ofstream traj_file(traj_fn);
			for (auto p:path){
				traj_file << p->get_x() << "," << p->get_y() << "," << p->get_theta() << endl;
			}
			traj_file.close();
			printf("finished writing\n");
			a_star->free_search_tree();
		}
	}


	delete a_star;
	delete ego_vehicle;
	delete env;
	return 0;
}