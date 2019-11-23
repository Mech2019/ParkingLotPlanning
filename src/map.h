#ifndef MAP_H
#define MAP_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#include "state.h"

static const char* map_name = "map.csv";
static double map_wid = 45.0;
static double map_len = 28.0;

class static_map{
private:
    double map_width;
    double map_length;
    int slot_num;
    std::vector<State*> slots;
public:
    static_map();
    static_map(double wid, double len, std::vector<std::vector<double>> input);
    ~static_map();
    int get_slot_num();
    double get_map_width();
    double get_map_length();
    std::vector<State*> get_slots();
};

void parse_static_map(const char* filename, std::vector<std::vector<double>>& input);

#endif