#include "map.h"

static_map::static_map(){}
static_map::static_map(double wid, double len, vector<vector<double>> input){
	this->map_width = wid;
	this->map_length = len;
	this->slot_num = input.size();
	for (int i = 0; i < input.size(); i++){
		slots.push_back(new State(input[i][0], input[i][1], input[i][2], false));
	}
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

void parse_static_map(char* filename, vector<vector<double>>& input){
	FILE *fp;
	fp = fopen(filename, "r");
	double x,y,theta;
	while (fscanf(fp, "%lf,%lf,%lf\n", &x, &y, &theta) == 3){
		input.push_back({x,y,theta});
	}
    fclose(fp);
}