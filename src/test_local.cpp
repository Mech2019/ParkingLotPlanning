#include "local_planner.h"
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
  printf("map size is %lfx%lf, total number of slots are %d\n",
         env->get_map_width(), env->get_map_length(), env->get_slot_num());

  cout << "test local planner..." << endl;

  CarState start(42.5354,  12.9548, 1.85903, 0, 0);
  CarState goal(48.25, 12.25, 0, 0, 0);

//  CarState goal(38, 8, 4.712389, 0, 0);

  vector<CarState> plan;

//  env->update_goal_list(&start);
//
//  State temp = State();
//
//  temp.set_x(start.get_x());
//  temp.set_y(start.get_y());
//  temp.set_theta(start.get_theta());
//
//  cout << "occupied_slots number: "
//  << env->get_occupied_slots().size() << endl;
//
//  bool res = collision_check(&temp, env->get_occupied_slots()[0]);


//  cout << "collision check: " << res << endl;

  local_planner(start, goal, plan, env);

}

