#ifndef MAP_H
#define MAP_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <string>
#include <fstream>
#include <sstream>

#include "state.h"
#include "util.h"

/*
 * this is the static map class that contains only static map info
 * fields are:
		map_width, map_length:      map physical dimension
		slot_num:                   total number of slots
		vector<State*> slots:       vector of all slots
		vector<State*> seen_slots:  vector of all seen slots by sensors
 * member functions:
		empty constructor
		constructor with three variable initialization
		destructor
		get functions
 */

class static_map{
private:
	double map_width;
	double map_length;
	int slot_num;
	std::vector<State*> slots;
	std::unordered_set<State*, StateHasher, StateComparator> seen_slots;
	std::vector<State*> unseen_slots;
public:
	static_map();
	static_map(double wid, double len, std::vector<std::vector<double>> input);
	~static_map();
	int get_slot_num();
	double get_map_width();
	double get_map_length();
	std::vector<State*> get_slots();
	std::unordered_set<State*, StateHasher, StateComparator> get_seen_slots();
	void update_seen_slots(CarState *ego);
};

void parse_static_map(const char* filename, std::vector<std::vector<double>>& input);

#endif