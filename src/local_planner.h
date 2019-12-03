

#ifndef PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H
#define PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H

#endif //PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <unordered_map>

#include "state.h"
#include "map.h"
#include "util.h"

void local_planner(CarState &start_state, CarState &goal_state,
                   vector<CarState>& plan);

class RRT_Tree {
public:
  int size;
  CarState start_pos;
  CarState goal_pos;
  unordered_map<int, CarState> graph;
  unordered_map<int, CarState> node_map;

  RRT_Tree(CarState start, CarState goal);
  void add_node(int id);
  void sample_node(int id);
};

