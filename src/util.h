#ifndef UTIL_H
#define UTIL_H

#include <math.h> 
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "state.h"

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

// global static variables
static const char *map_name = "map.csv";
static const double slot_wid = 2.5;
static const double slot_dep = 5.5;
static const double map_wid = 45.0;	// update this value with the map
static const double map_len = 28.0;
static const double sensor_range = 10.0;
static const double car_wid = 1.8;
static const double car_len = 4.5;


/* helper function for collision check */
bool intersect_point(State *s1, double x, double y);
bool onSegment(double x1, double y1, double x2, double y2, double x3, double y3);
int orientation(double x1, double y1, double x2, double y2, double x3, double y3);
bool intersect_lines(double x1, double y1, double x2, double y2, 
	double x3, double y3, double x4, double y4);
// make collision_check a template function so that car_state can iteract with state
// template <class T*>
bool collision_check(State *s1, State *s2);

#endif