
#include "local_planner.h"

RRT_Tree::RRT_Tree(CarState start, CarState goal) {
  start_pos = start;
  goal_pos = goal;
  node_map[0] = start_pos;
  graph[0] = {};
  size = 1;
}

void RRT_Tree::sample_node(int id, CarState& rand_state) {
  srand(id); // fixed for debug reason

  double rand_x, rand_y, rand_theta;

  rand_x = rand() % MAP_WIDTH;
  rand_y = rand() % MAP_HEIGHT;
  rand_theta = rand() % (int) (2 * M_PI / RANDOM_STEP) * RANDOM_STEP;

  rand_state.set_x(rand_x);
  rand_state.set_y(rand_y);
  rand_state.set_theta(rand_theta);

  cout << "Sample from random: " << rand_state << endl;
}

void RRT_Tree::sample_node_from_primitives(int id, CarState &rand_state,
    CarState& curr_state) {

  vector<vector<CarState>> result;
  vector<State*> obstacle;
  curr_state.compute_primitive(result, obstacle);

  int m = result.size();
  int n = result[0].size();
  cout << "sample size = " << m << ", " << n << endl;

  srand(id); // fixed for debug reason
  int selected_m = rand() % m;
  int selected_n = rand() % n;

  rand_state = result[selected_m][selected_n];
  cout <<"Sample from primitives result: " << rand_state << endl;

}

void RRT_Tree::extend(CarState& rand_state) {

  CarState nearest = CarState();
  nearest_neighbor(rand_state, nearest);

  cout << "in extend, found nearest " << nearest << endl;

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

void RRT_Tree::nearest_neighbor(CarState& rand_state, CarState& nearest){

  double nearest_dist = INT_MAX;
  double curr_dist;
  int nearest_id;

  for (int i =0; i<size; i++){
    cout << "node: " << node_map[i] << endl;
//    calculate_distance
    curr_dist = calculate_distance(node_map[i], rand_state);
    if (curr_dist < nearest_dist){
      nearest_dist = curr_dist;
      nearest_id = i;
    }
  }

  cout << "neareset node id " << nearest_id << ", " << nearest_dist << endl;
  nearest = node_map[nearest_id];
}

void RRT_Tree::add_node(int id) {

  CarState rand_state = CarState();
  sample_node(id, rand_state);
  extend(rand_state);



}

void RRT_Tree::add_node_from_primitives(int id, CarState& curr_state) {

  CarState nearest = CarState();
  CarState rand_state = CarState();

  sample_node_from_primitives(id, rand_state, curr_state);

  extend(rand_state);
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

void local_planner(CarState &start, CarState &goal,
    vector<CarState>& plan){

  cout << "start local planner:" << endl;
  cout << "start " << start << endl;
  cout << "goal " << goal << endl;

  RRT_Tree tree(start, goal);

//  tree.add_node(1);
  tree.add_node_from_primitives(1, start);
  tree.print_tree();









  // open a file in write mode.
  ofstream outfile;
  outfile.open("local_result.txt");
  outfile << start << endl;

  outfile << goal << endl;
  outfile.close();


}


