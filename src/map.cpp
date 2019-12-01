#include "map.h"

using namespace std;

static_map::static_map(){}
static_map::static_map(double wid, double len, vector<vector<double>> input){
	this->map_width = wid;
	this->map_length = len;
	this->slot_num = input.size();
	for (int i = 0; i < input.size(); i++){
		slots.push_back(new State(input[i][0], input[i][1], input[i][2], input[i][3]));
	}
	unseen_slots = slots;
}
static_map::~static_map(){
	for (int i = 0; i < slots.size(); i++){
		delete slots[i];
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
vector<State*> static_map::get_seen_slots(){
	return this->seen_slots;
}
/*
 * This function takes in the ego vehicle state and update seen_slots
 */
void static_map::update_seen_slots(CarState *ego){
	double ego_x = ego->get_x();
	double ego_y = ego->get_y();
	auto it = unseen_slots.begin(); 
	while (it! = unseen_slots.end()){
		State *slot = *it;
		double slot_x = slot->get_x();
		double slot_y = slot->get_y();
		double dx = ego_x - slot_x;
		double dy = ego_y - slot_y;
		// skip slots outside the sensor range
		if (sqrt(dx*dx + dy*dy) > sensor_range)
			continue;
		// skip slots that are unseen
		if (seen_slots.find(slot) != seen_slots.end())
			continue;
		// check for obstacles 
		bool obstructed = false;
		for (int j = 0; j < this->slot_num; j++){
			State *tmp = slots[j];
			double tmp_x = tmp->get_x();
			double tmp_y = tmp->get_y();
			if (!tmp->get_flag())	// do not need to check empty one
				continue;
			// inflate the slot into rectangles
			double llx = tmp_x - car_len/2, lly = tmp_y - car_wid/2;
			double ulx = tmp_x - car_len/2, uly = tmp_y + car_wid/2;
			double lrx = tmp_x + car_len/2, lry = tmp_y - car_wid/2;
			double urx = tmp_x + car_len/2, ury = tmp_y + car_wid/2;
			// perform line of sight check here
		}
		// remove from unseen_slots if seen
		// add to seen_slots if seen
		if (!obstructed){
			unseen_slots.erase(it);
			seen_slots.push_back(slot);	
		} else {
			it++;
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
	fp = fopen(filename, "r");
	double x,y,theta;
	int full;
	while (fscanf(fp, "%lf,%lf,%lf,%d\n", &x, &y, &theta, &full) == 3){
		input.push_back({x,y,theta,full});
	}
	fclose(fp);
}