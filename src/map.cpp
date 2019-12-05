#include "map.h"

using namespace std;

static_map::static_map(){}
static_map::static_map(double wid, double len, vector<vector<double>> input){
  this->map_width = wid;
  this->map_length = len;
  this->slot_num = input.size();
  for (int i = 0; i < input.size(); i++){
    // slots contain all slots with true information
    // slots.push_back(shared_ptr<State>(new State(input[i][0], input[i][1], input[i][2], input[i][3])));
    slots.push_back(new State(input[i][0], input[i][1], input[i][2], input[i][3]));
    // update occupied slot list
    if (input[i][3])
      occupied_slots.push_back(slots[i]);
    // goal list initially contains slots with assumed information
    // goal_list.insert(shared_ptr<State>(new State(input[i][0], input[i][1], input[i][2], false)));
    goal_list.insert(new State(input[i][0], input[i][1], input[i][2], false));
  }
  unseen_slots = slots;
}
static_map::~static_map(){
  for (int i = 0; i < slots.size(); i++){
    delete slots[i];
  }
  for (auto it = goal_list.begin(); it != goal_list.end(); it++){
    delete(*it);
  }
}
int static_map::get_slot_num(){
  return this->slot_num;
}
double static_map::get_map_width(){
  return this->map_width;
}
double static_map::get_map_length(){
  return this->map_length;
}
vector<State*> static_map::get_slots(){
  return this->slots;
}
vector<State*> static_map::get_occupied_slots(){
  return this->occupied_slots;
}
unordered_set<State*, StateHasher, StateComparator> static_map::get_goal_list(){
  return this->goal_list;
}
/*
 * This function takes in the ego vehicle state and update seen_slots
 */
void static_map::update_goal_list(CarState *ego){
  double ego_x = ego->get_x();
  double ego_y = ego->get_y();
  auto it = unseen_slots.begin();
  while (it != unseen_slots.end()){
    State *slot = *it;
    double slot_x = slot->get_x();
    double slot_y = slot->get_y();
    double dx = ego_x - slot_x;
    double dy = ego_y - slot_y;
    // skip slots outside the sensor range
    if (sqrt(dx*dx + dy*dy) > sensor_range){
      ++it;
      continue;
    }
    // check for obstacles
    bool obstructed = false;
    // printf("************\n");
    for (int j = 0; j < this->slot_num && !obstructed; j++){
      // printf("j = %d\n", j);
      State *tmp = slots[j];
      double tmp_x = tmp->get_x();
      double tmp_y = tmp->get_y();
      if (!tmp->get_flag())	// do not need to check empty one
        continue;
      if (tmp == slot)		// no need to check for self
        continue;
      // no need to check slots not b/t slot and ego
      if ((tmp_x - ego_x)*(tmp_x - slot_x) > 0
          || (tmp_y - ego_y)*(tmp_y - slot_y) > 0)
        continue;
      // inflate the slot into rectangles
      double llx = tmp_x - car_len/2, lly = tmp_y - car_wid/2;
      double ulx = tmp_x - car_len/2, uly = tmp_y + car_wid/2;
      double lrx = tmp_x + car_len/2, lry = tmp_y - car_wid/2;
      double urx = tmp_x + car_len/2, ury = tmp_y + car_wid/2;
      vector<double> x_coord = {llx, ulx, urx, lrx, llx};
      vector<double> y_coord = {lly, uly, ury, lry, lly};
      // perform line of sight check here
      // first calculate the line expression of the ego to the target
      if (!slot->get_flag()){	// empty slot
        for (int k = 0; k < 4 && !obstructed; k++){
          double x1 = slot_x, x2 = ego_x;
          double y1 = slot_y, y2 = ego_y;
          double x3 = x_coord[k], x4 = x_coord[k+1];
          double y3 = y_coord[k], y4 = y_coord[k+1];
          obstructed = isLineofSightObstructed(x1, x2, x3, x4, y1, y2, y3, y4);
        }
      } else {	// occupied slot, if we can clearly see one vertices, then we are good
        double slot_llx = slot_x - car_len/2, slot_lly = slot_y - car_wid/2;
        double slot_ulx = slot_x - car_len/2, slot_uly = slot_y + car_wid/2;
        double slot_lrx = slot_x + car_len/2, slot_lry = slot_y - car_wid/2;
        double slot_urx = slot_x + car_len/2, slot_ury = slot_y + car_wid/2;
        vector<double> slot_x_coord = {slot_llx, slot_ulx, slot_urx, slot_lrx};
        vector<double> slot_y_coord = {slot_lly, slot_uly, slot_ury, slot_lry};
        for (int m = 0; m < 4; m++){
          double x1 = slot_x_coord[m], x2 = ego_x;
          double y1 = slot_y_coord[m], y2 = ego_y;
          for (int k = 0; k < 4 && !obstructed; k++){
            double x3 = x_coord[k], x4 = x_coord[k+1];
            double y3 = y_coord[k], y4 = y_coord[k+1];
            obstructed = isLineofSightObstructed(x1, x2, x3, x4, y1, y2, y3, y4);
          }
          if (!obstructed){
            break;
          }
        }
      }


    }
    // remove from unseen_slots if seen
    // add to seen_slots if seen
    if (!obstructed){
      // printf("erased\n");
      unseen_slots.erase(it);
      // if this slot is not empty, erase it from the goal list
      if ((slot->get_flag())){
//        printf("state to erase: \n");
//        cout << slot << endl;
        auto goal_it = goal_list.find(slot);
        if (goal_it != goal_list.end()){
          // printf("found the slot in the goal list\n");
          goal_list.erase(goal_it);
        }
      }
    } else {
      ++it;
    }
  }
}
/*
 * This function parses the static map created by Matlab
 * the incoming map should have four columns:
 		x 		coordinate
 		y 		coordinate
 		theta	heading angle
 		full	indicator -- 1 for full, 0 for empty
 * inputs: map filename, input vector waiting to be changed
 * output: none
 */
void parse_static_map(const char* filename, vector<vector<double>>& input){
  FILE *fp;
  if (!(fp = fopen(filename, "r"))){
    perror("fopen error");
  }
  double x,y,theta,full;
  while (fscanf(fp, "%lf,%lf,%lf,%lf\n", &x, &y, &theta, &full) == 4){
    input.push_back({x,y,theta,full});
  }
  fclose(fp);
}


bool isLineofSightObstructed(double x1, double x2, double x3, double x4,
                             double y1, double y2, double y3, double y4){
  if (x1 == x2 && x3 == x4){
    if (x1 == x3){
      return true;
    }
  } else if (x1 == x2 && x3 != x4) {
    double m2 = (y4 - y3) / (x4 - x3);
    double b2 = y3 - m2 * x3;

    double x_intersect = x1;
    double y_intersect = m2 * x_intersect + b2;
    if ((x_intersect-x1)*(x_intersect-x2) <= 0
        && (x_intersect-x3)*(x_intersect-x4) <= 0
        && (y_intersect-y1)*(y_intersect-y2) <= 0
        && (y_intersect-y3)*(y_intersect-y4) <= 0){
      return true;
    }
  } else if (x1 != x2 && x3 == x4) {
    double m1 = (y2 - y1) / (x2 - x1);
    double b1 = y1 - m1 * x1;

    double x_intersect = x3;
    double y_intersect = m1 * x_intersect + b1;
    if ((x_intersect-x1)*(x_intersect-x2) <= 0
        && (x_intersect-x3)*(x_intersect-x4) <= 0
        && (y_intersect-y1)*(y_intersect-y2) <= 0
        && (y_intersect-y3)*(y_intersect-y4) <= 0){
      return true;
    }
    // printf("case3, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
  } else {
    // calculate slope and y-intersection for line
    double m1 = (y2 - y1) / (x2 - x1);
    double b1 = y1 - m1 * x1;
    double m2 = (y4 - y3) / (x4 - x3);
    double b2 = y3 - m2 * x3;
    // printf("line 1: m1 = %lf, b1 = %lf\n", m1, b1);
    // printf("line 2: m2 = %lf, b2 = %lf\n", m2, b2);
    // calculate intersection coordinate
    double x_intersect = (b2-b1)/(m1-m2);
    double y_intersect = m1 * x_intersect + b1;
    // check if the intersection point is on both segment
    if ((x_intersect-x1)*(x_intersect-x2) <= 0
        && (x_intersect-x3)*(x_intersect-x4) <= 0
        && (y_intersect-y1)*(y_intersect-y2) <= 0
        && (y_intersect-y3)*(y_intersect-y4) <= 0){
      return true;
    }
    // printf("case4, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
  }
  return false;
}