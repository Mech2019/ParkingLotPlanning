#ifndef MAP_H
#define MAP_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

static const char* map_name = "map.csv";

class static_map{
private:
    double map_width;
    double map_length;
    int slot_num;
    vector<State*> slots;
public:
    static_map(){}
    static_map(double wid, double len, vector<vector<double>> input);
    ~static_map();
    int get_slot_num();
    double get_map_width();
    double get_map_length();
    vector<State*> get_slots();
};

void parse_static_map(char* filename, vector<vector<double>>& input);

#endif