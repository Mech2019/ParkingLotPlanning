#include "planner.h"

using namespace std;

int main(){
	/* test map parsing */
	vector<vector<double>> input;
	parse_static_map(map_name, input);
	static_map map(map_wid, map_len, input);
	// print to see map info
	printf("map size is %lfx%lf, total number of spots are %d\n", 
		map.get_map_width(), map.get_map_length(), map.get_slot_num());
	
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
	
	return 0;
}