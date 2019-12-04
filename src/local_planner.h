
#ifndef PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H
#define PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H


#include <iostream>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <math.h>

#include "state.h"
#include "map.h"
#include "util.h"


#define MAP_WIDTH 40   // for temp use
#define MAP_HEIGHT 28


const double EPSILON_DIST = 0.15;
const double EPSILON_THETA = M_PI/60;
const double RANDOM_STEP = M_PI/20;
const double GOAL_THRESHOLD = 0.2;
const double GOAL_BIAS = 0.05;
const double TURN_RANGE = M_PI / 3;
const double TURN_MIN = (M_PI * 5 / 180);

void local_planner(CarState &start_state, CarState &goal_state,
                   vector<CarState>& plan);


class RRT_Tree {
public:
  int size;
  CarState start_pos;
  CarState goal_pos;
  unordered_map<int, vector<int>> graph;
  unordered_map<int, CarState> node_map;
  unordered_map<int, int> parent_node;
  double min_dist;
  bool reached;

  RRT_Tree(CarState start, CarState goal);
  void add_node(int id);
  void add_node_from_primitives(int id, CarState& curr_state);
  void sample_node(int id, CarState& rand_state);
  void sample_node_from_primitives(int id, CarState& rand_state, CarState&
  curr_state);
  void extend(CarState& rand_state);
  int nearest_neighbor(CarState& rand_state, CarState& nearest);
  void get_new_state_from_nearest(CarState& rand_state, CarState& nearest,
      CarState& new_state);

  void print_tree();
  double calculate_distance(CarState& from_state, CarState& to_state);
  bool check_if_reached(CarState& from, CarState& goal);
  void reconstruct_path(int last_node_id, vector<int> &path_id);
  bool local_collision_check(CarState& curr_state, static_map *env);
};



#endif //PARKINGLOTPLANNING_SRC_LOCAL_PLANNER_H

