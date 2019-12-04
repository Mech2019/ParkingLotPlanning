
#include "local_planner.h"

RRT_Tree::RRT_Tree(CarState start, CarState goal) {
  start_pos = start;
  goal_pos = goal;
  node_map[0] = start_pos;
  graph[0] = {};
  size = 1;
  min_dist = INT_MAX;
  reached = 0;
}

void RRT_Tree::sample_node(int id, CarState& rand_state) {
  srand(id); // fixed for debug reason

  double rand_x, rand_y, rand_theta;

  int rand_for_bias = rand() % 100;

  if (rand_for_bias < (GOAL_BIAS * 100)) {

    rand_x = goal_pos.get_x() - start_pos.get_x();
    rand_y = goal_pos.get_y() - start_pos.get_y();
    rand_theta =  goal_pos.get_theta() - start_pos.get_theta();
//    cout << "BIASED!" << endl;

  } else {
    rand_x = rand() % MAP_WIDTH - MAP_WIDTH/2;
    rand_y = rand() % MAP_HEIGHT - MAP_HEIGHT/2;
    rand_theta = rand() % (int) (2 * M_PI / RANDOM_STEP) * RANDOM_STEP - M_PI;

//    cout << "rand theta: " << rand_theta << endl;
  }



  rand_state.set_x(rand_x);
  rand_state.set_y(rand_y);
  rand_state.set_theta(rand_theta);

//  cout << "Sample from random: " << rand_state << endl;
}

void RRT_Tree::sample_node_from_primitives(int id, CarState &rand_state,
    CarState& curr_state) {

  vector<vector<CarState>> result;
  vector<State*> obstacle;
  curr_state.compute_primitive(result, obstacle);

  int m = result.size();
  int n = result[0].size();
//  cout << "sample size = " << m << ", " << n << endl;

  srand(id); // fixed for debug reason
  int selected_m = rand() % m;
  int selected_n = rand() % n;

  rand_state = result[selected_m][selected_n];
//  cout <<"Sample from primitives result: " << rand_state << endl;

}

void RRT_Tree::get_new_state_from_nearest(CarState& rand_state, CarState& nearest,
                                CarState& new_state){

//  new_state.set_x(nearest.get_x() +
//  EPSILON_DIST * (rand_state.get_x() - nearest.get_x()));
//
//  new_state.set_y(nearest.get_y() +
//  EPSILON_DIST * (rand_state.get_y() - nearest.get_y()));
//
//  new_state.set_theta(nearest.get_theta() +
//  EPSILON_THETA * (rand_state.get_theta() - nearest.get_theta()));

  double d_x, d_y, d_theta;

  if (rand_state.get_x() >= 0){
    d_x = MIN(EPSILON_DIST, rand_state.get_x());
  } else {
//    cout << "check01" << endl;
    d_x = MAX(-EPSILON_DIST, rand_state.get_x());
  }


  if (rand_state.get_y() >=0){
    d_y = MIN(EPSILON_DIST, rand_state.get_y());
  } else {
    d_y = MAX(-EPSILON_DIST, rand_state.get_y());
  }

  if (rand_state.get_theta() >= 0){
    d_theta = MIN(EPSILON_THETA, rand_state.get_theta());
  } else {
    d_theta = MAX(-EPSILON_THETA, rand_state.get_theta());
  }

//  cout << d_x << ", " << d_y << ", " << d_theta << endl;

  new_state.set_x(nearest.get_x() - d_x);
  new_state.set_y(nearest.get_y() - d_y);
  new_state.set_theta(nearest.get_theta() - d_theta);

  cout << "neareset : " << nearest << endl;
  cout << "random   :" << rand_state << endl;
  cout << "new      :" << new_state << endl;
}


double RRT_Tree::calculate_distance(CarState& from_state, CarState& to_state){

  double dist = 0;
  double x1 = from_state.get_x();
  double y1 = from_state.get_y();
  double x2 = to_state.get_x();
  double y2 = to_state.get_y();
  dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
  return dist;
}

int RRT_Tree::nearest_neighbor(CarState& rand_state, CarState& nearest){

  double nearest_dist = INT_MAX;
  double curr_dist;
  int nearest_id;

  int temp_node_size = node_map.size();

  for (auto map : node_map){
//    calculate_distance
    int temp_id = map.first;
    CarState temp_state = map.second;

    cout << "  temp state: " << temp_state << endl;
    cout << "  rand state: " << rand_state << endl;
    curr_dist = calculate_distance(temp_state, rand_state);

    if (curr_dist < nearest_dist){
      nearest_dist = curr_dist;
      nearest_id = temp_id;
    }

    cout << "  Map node: " << temp_id << ", dist: ";
    cout <<curr_dist << ", " << nearest_dist << ", "<< nearest_id << endl;

  }


//  cout << "neareset node id " << nearest_id << ", " << nearest_dist << endl;
  nearest = node_map[nearest_id];
//  cout << "IN nearest: " << nearest << endl;
  return nearest_id;
}


void RRT_Tree::extend(int id, CarState& rand_state, static_map *env) {

  CarState nearest = CarState();
  CarState new_state = CarState();

  int nearest_id = nearest_neighbor(rand_state, nearest);
//  cout << "in extend, found nearest " << nearest << endl;

  get_new_state_from_nearest(rand_state, nearest, new_state);
//  cout << "new node : " << new_state << endl;

  bool collision_check_res = local_collision_check(new_state, env);
//  cout << "_____collision check: " << collision_check_res << endl;

  if (!collision_check_res || 1){

    // if no collision, add node
    int new_id = id;
    node_map[new_id] = new_state;
    graph[nearest_id].push_back(new_id);
    graph[new_id] = {};
    parent_node[new_id] = nearest_id;
    size++;

    // check if reached
    if (check_if_reached(new_state, goal_pos)){
      reached = 1;
      cout << "reached!" << endl;
    }
  }



}


void RRT_Tree::add_node(int id, static_map *env) {

  CarState rand_state = CarState();
  sample_node(id, rand_state);
  extend(id, rand_state, env);

}

void RRT_Tree::add_node_from_primitives(int id, CarState& curr_state, static_map *env) {

  CarState nearest = CarState();
  CarState rand_state = CarState();

  sample_node_from_primitives(id, rand_state, curr_state);

  extend(id, rand_state, env);
  nearest_neighbor(rand_state, nearest);

  sample_node_from_primitives(id, rand_state, curr_state);

}

void RRT_Tree::print_tree() {

  int n = graph.size();

  cout << "-------------------current graph----------------------" << endl;
  if (n == 0) {
    cout << "Warning: nothing in tree!" << endl;
  } else {

    for (auto it = graph.begin(); it != graph.end(); it++) {
      cout << it->first << ": {";
      auto next = it->second;
      for (auto vec_it = next.begin(); vec_it != next.end(); vec_it++) {
        cout << *vec_it << ", ";
      }
      cout << " }" << endl;
    }
  }
  cout << "------------------------------------------------------" << endl;
}


bool RRT_Tree::check_if_reached(CarState& from, CarState& goal){
  double curr_dist = calculate_distance(from, goal);
  if (curr_dist < min_dist){
    min_dist = curr_dist;
    cout << "min dist = " << min_dist << endl;
  }

  double curr_theta = from.get_theta();
  double goal_theta = goal.get_theta();
  double theta_diff = abs(curr_theta - goal_theta);

  if (curr_dist < GOAL_THRESHOLD && theta_diff < GOAL_THETA_THRESHOLD){
    return true;
  }
  return false;
}

void RRT_Tree::reconstruct_path(int last_node_id, vector<int> &path_id) {
  if (last_node_id == 0) {
    cout << "already in goal position with threshold setting!\n";
  }

  // push back the given node id
  path_id.push_back(last_node_id);

  int parent_id = parent_node[last_node_id];

  while (parent_id != 0) {
    path_id.push_back(parent_id);
    parent_id = parent_node[parent_id];
  }
//    path_id.push_back(0);
  reverse(begin(path_id), end(path_id));

  cout << "reconstruct path is: ";
  for (auto v : path_id){
    cout << v << ", ";
  }
  cout << endl;

}

bool local_collision_check(CarState &curr_state, static_map *env) {
  env->update_goal_list(&curr_state);
  vector<State*> occupied_slots =  env->get_occupied_slots();
  int obs_size =occupied_slots.size();
//  cout << "occupied_slots number: " << n << endl;

  for (int i=0; i<obs_size; i++){
    State temp = State();

    temp.set_x(curr_state.get_x());
    temp.set_y(curr_state.get_y());
    temp.set_theta(curr_state.get_theta());

    if (collision_check(occupied_slots[i], &temp)) {
//      cout << "COLLISION for: " << endl;
//      cout << (occupied_slots[i]);
//      cout << " and " << curr_state <<endl;
      return true;
    }
  }

  return false;
}





void local_planner(CarState &start, CarState &goal,
    vector<CarState>& plan, static_map *env){

  cout << "start local planner:" << endl;
  cout << "start " << start << endl;
  cout << "goal " << goal << endl;

  local_collision_check(start, env);
  local_collision_check(goal, env);

  // open a file in write mode.
  ofstream outfile;
  outfile.open("local_result.txt");
  outfile << start << endl;

  bool reached = 0;
  vector<int> path_id;

  RRT_Tree tree(start, goal);
  for (int i = 1; i < SAMPLES; i++){
    tree.add_node(i, env);
    if (tree.reached){
      tree.reconstruct_path(i, path_id);
      break;
    }
  }

  for (auto id : path_id){
    outfile << tree.node_map[id] << endl;
  }


//  tree.add_node_from_primitives(1, start);
//  tree.add_node_from_primitives(2, start);
//  tree.print_tree();


  cout << "node map size = " << tree.node_map.size() << endl;

  for (int i=0; i < tree.node_map.size(); i++){
//    outfile << tree.node_map[i] << endl;
//    cout << tree.node_map[i] << endl;
  }

  outfile << goal << endl;
  outfile.close();


}






