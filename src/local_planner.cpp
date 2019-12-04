
#include "local_planner.h"

RRT_Tree::RRT_Tree(CarState start, CarState goal) {
  start_pos = start;
  goal_pos = goal;
  node_map[0] = start_pos;
}

void RRT_Tree::sample_node(int id) {

}

void RRT_Tree::add_node(int id) {

}

void local_planner(CarState &start, CarState &goal,
    vector<CarState>& plan){

  cout << "start local planner:" << endl;
  cout << start << endl;
  cout << goal << endl;

  RRT_Tree tree(start, goal);
  tree.add_node(1);








  // open a file in write mode.
  ofstream outfile;
  outfile.open("local_result.txt");
  outfile << start << endl;

  outfile << goal << endl;
  outfile.close();


}


