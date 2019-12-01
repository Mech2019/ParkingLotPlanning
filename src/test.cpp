#include "planner.h"

using namespace std;

int main(){
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
	

	/* test collision checking */
	//  printf("Start check collision !\n");
	State *state1 = new State(0, 0, 0, 0);
	State *state2 = new State(10, 0, 0, 0);
	cout << collision_check(state1, state2) << endl;

	State *state3 = new State(0, 0, 0, 0);
	State *state4 = new State(5, 3, 0.54, 0);
	cout << collision_check(state3, state4) << endl;

	delete(state1);
	delete(state2);
	delete(state3);
	delete(state4);
	
	/* test sensor reading */
	CarState *ego_vehicle = new CarState(7.5, 2.75, PI/2, 1, 0.0);
	env->update_seen_slots(ego_vehicle);
	auto seen_slots = env->get_seen_slots();
	for (auto it = seen_slots.begin(), it != seen_slots.end(); it++){
		auto slot = *it;
		cout << slot << endl;
	}

	delete ego_vehicle;
	delete env;
	return 0;
}