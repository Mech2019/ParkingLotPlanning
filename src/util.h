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


#define PI 3.1415927

#if !defined(TORAD)
#define	TORAD(A)	((A)*3.14159265359/180.0)
#endif

#if !defined(TODEG)
#define	TODEG(A)	((A)/3.14159265359*180.0)
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
template <class T>
bool collision_check(CarState *s1, T *s2){
	int direc[] = {1, 1, -1, -1, 1}; 

	std::vector<double> v1, v2;
	for (int i = 0; i < 4; i++) {
		v1.push_back(s1->get_x() + direc[i] * (car_wid * sin(s1->get_theta()) + car_len * cos(s1->get_theta())));
		v1.push_back(s1->get_y() + direc[i + 1] * (car_wid * cos(s1->get_theta()) + car_len * sin(s1->get_theta())));
		v2.push_back(s2->get_x() + direc[i] * (car_wid * sin(s2->get_theta()) + car_len * cos(s2->get_theta())));
		v2.push_back(s2->get_y() + direc[i + 1] * (car_wid * cos(s2->get_theta()) + car_len * sin(s2->get_theta())));
	}

	v1.push_back(s1->get_x() + (car_wid * sin(s1->get_theta()) + car_len * cos(s1->get_theta())));
	v1.push_back(s1->get_y() + (car_wid * cos(s1->get_theta()) + car_len * sin(s1->get_theta())));
	v2.push_back(s2->get_x() + (car_wid * sin(s2->get_theta()) + car_len * cos(s2->get_theta())));
	v2.push_back(s2->get_y() + (car_wid * cos(s2->get_theta()) + car_len * sin(s2->get_theta())));

	for (int i = 0; i < 4; i++) {
		if (intersect_point(s1, v2[2 * i], v2[2 * i + 1])) {
		return true;
		}
	}
	
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (intersect_lines(v1[2 * i], v1[2 * i + 1], v1[2 * (i + 1)], v1[2 * (i + 1) + 1], 
				v2[2 * i], v2[2 * i + 1], v2[2 * (i + 1)], v2[2 * (i + 1) + 1])) {
				return true;
			}
		}
	}
	return false;
}
// bool collision_check(State *s1, State *s2);

#endif