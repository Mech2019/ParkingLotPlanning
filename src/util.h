#ifndef UTIL_H
#define UTIL_H

#include <math.h> 
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "state.h"

#if !defined(PI)
#define	PI 3.14159265359
#endif

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#if !defined(TORAD)
#define	TORAD(A)	((A)*PI/180.0)
#endif

#if !defined(TODEG)
#define	TODEG(A)	((A)/PI*180.0)
#endif

// global static variables
static const char *map_name = "map.csv";
static const double slot_wid = 2.5;
static const double slot_dep = 5.5;
static const double map_wid = 51.0;	// update this value with the map
static const double map_len = 32.0;
static const double sensor_range = 10.0;
static const double car_wid = 1.8;
static const double car_len = 4.5;
static const double car_speed = 5.0; // in m/s
static const double wheel_base = 2.7; // distance between front and back wheel axis

// motion primitive variables
const double DURATION = 1.0; // time to drive
const int SAMPLE_POINTS = 20; // sampled points per each primitive
const double delta_MAX = TORAD(30); // max steering angle to right
const double delta_MIN = TORAD(-30); // max steering angle to left

/* helper function for collision check */
// bool intersect_point(State *s1, double x, double y);
// bool onSegment(double x1, double y1, double x2, double y2, double x3, double y3);
// int orientation(double x1, double y1, double x2, double y2, double x3, double y3);
// bool intersect_lines(double x1, double y1, double x2, double y2, 
// 	double x3, double y3, double x4, double y4);
bool linesegmentcheck(double x1, double x2, double x3, double x4,
								double y1, double y2, double y3, double y4);
template <class T>
vector<vector<double>> inflate(T *s){
	double theta = s->get_theta();
	// ll, ul, ur, lr, ll
	vector<double> x_coord = {-car_len/2, -car_len/2, car_len/2,  car_len/2, -car_len/2};
	vector<double> y_coord = {-car_wid/2,  car_wid/2, car_wid/2, -car_wid/2, -car_wid/2};

	vector<vector<double>> result(2, vector<double>(5, 0.0));

	for (int i = 0; i < 5; i++){
		result[0][i] = cos(theta) * x_coord[i] - sin(theta) * y_coord[i] + s->get_x();
		result[1][i] = sin(theta) * x_coord[i] + cos(theta) * y_coord[i] + s->get_y();
	} 
	return result;
}

template <class T>
bool collision_check(State *s1, T *s2){
	// int direc[] = {1, 1, -1, -1, 1}; 

	// std::vector<double> v1, v2;
	// for (int i = 0; i < 4; i++) {
	// 	v1.push_back(s1->get_x() + direc[i] * (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
	// 	v1.push_back(s1->get_y() + direc[i + 1] * (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
	// 	v2.push_back(s2->get_x() + direc[i] * (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
	// 	v2.push_back(s2->get_y() + direc[i + 1] * (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));
	// }

	// v1.push_back(s1->get_x() + (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
	// v1.push_back(s1->get_y() + (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
	// v2.push_back(s2->get_x() + (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
	// v2.push_back(s2->get_y() + (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));

	// for (int i = 0; i < 4; i++) {
	// 	if (intersect_point(s1, v2[2 * i], v2[2 * i + 1])) {
	// 	return true;
	// 	}
	// }
	// for (int i = 0; i < 4; i++) {
	// 	for (int j = 0; j < 4; j++) {
	// 		if (intersect_lines(v1[2 * i], v1[2 * i + 1], v1[2 * (i + 1)], v1[2 * (i + 1) + 1], 
	// 			v2[2 * i], v2[2 * i + 1], v2[2 * (i + 1)], v2[2 * (i + 1) + 1])) {
	// 			return true;
	// 		}
	// 	}
	// }
/*************************************/
	// inflate state 1 and state 2
	vector<vector<double>> inflated_s1 = inflate(s1);
	vector<vector<double>> inflated_s2 = inflate(s2);
	// check line segment
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			double x1 = inflated_s1[0][i], x2 = inflated_s1[0][i+1];
			double y1 = inflated_s1[1][i], y2 = inflated_s1[1][i+1];

			double x3 = inflated_s2[0][j], x4 = inflated_s2[0][j+1];
			double y3 = inflated_s2[1][j], y4 = inflated_s2[1][j+1];

			// printf("%lf, %lf, %lf, %lf\n", x1,y1,x2,y2);
			// printf("%lf, %lf, %lf, %lf\n", x3,y3,x4,y4);
			// printf("result = %d\n", linesegmentcheck(x1,x2,x3,x4,y1,y2,y3,y4));
			if (linesegmentcheck(x1,x2,x3,x4,y1,y2,y3,y4) == true)
				return true;
		}
	}
	// out of bound check
	// if (s1->get_x() < 0 || s1->get_x() > map_wid)
	// 	return true;
	// if (s2->get_x() < 0 || s2->get_x() > map_wid)
	// 	return true;
	// if (s1->get_y() < 0 || s1->get_y() > map_len)
	// 	return true;
	// if (s2->get_y() < 0 || s2->get_y() > map_len)
	// 	return true;
	return false;
}
// std::vector<std::vector<double>> inflate(T *s);
// // make collision_check a template function so that car_state can iteract with state
// template <class T>
// bool collision_check(State *s1, T *s2){
// 	int direc[] = {1, 1, -1, -1, 1}; 

// 	std::vector<double> v1, v2;
// 	for (int i = 0; i < 4; i++) {
// 		v1.push_back(s1->get_x() + direc[i] * (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
// 		v1.push_back(s1->get_y() + direc[i + 1] * (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
// 		v2.push_back(s2->get_x() + direc[i] * (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
// 		v2.push_back(s2->get_y() + direc[i + 1] * (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));
// 	}

// 	v1.push_back(s1->get_x() + (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
// 	v1.push_back(s1->get_y() + (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
// 	v2.push_back(s2->get_x() + (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
// 	v2.push_back(s2->get_y() + (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));
	
// 	// printf("%lf, %lf, %lf, %lf\n", v1[0],v1[1],v1[2],v1[3]);
// 	// printf("%lf, %lf, %lf, %lf\n", v2[0],v2[1],v2[2],v2[3]);
	
// 	for (int i = 0; i < 4; i++) {
// 		if (intersect_point(s1, v2[2 * i], v2[2 * i + 1])) {
// 		return true;
// 		}
// 	}
// 	for (int i = 0; i < 4; i++) {
// 		for (int j = 0; j < 4; j++) {
// 			if (intersect_lines(v1[2 * i], v1[2 * i + 1], v1[2 * (i + 1)], v1[2 * (i + 1) + 1], 
// 				v2[2 * i], v2[2 * i + 1], v2[2 * (i + 1)], v2[2 * (i + 1) + 1])) {
// 				return true;
// 			}
// 		}
// 	}

// 	// out of bound check
// 	// if (s1->get_x() < 0 || s1->get_x() > map_wid)
// 	// 	return true;
// 	if (s2->get_x() < 0 || s2->get_x() > map_wid)
// 		return true;
// 	// if (s1->get_y() < 0 || s1->get_y() > map_len)
// 	// 	return true;
// 	if (s2->get_y() < 0 || s2->get_y() > map_len)
// 		return true;
// 	return false;
// }



// template <class T>
// bool collision_check(State *s1, T *s2){
// 	int direc[] = {1, 1, -1, -1, 1}; 

// 	std::vector<double> v1, v2;
// 	for (int i = 0; i < 4; i++) {
// 		v1.push_back(s1->get_x() + direc[i] * (car_wid * sin(s1->get_theta()) + car_len * cos(s1->get_theta())));
// 		v1.push_back(s1->get_y() + direc[i + 1] * (car_wid * cos(s1->get_theta()) + car_len * sin(s1->get_theta())));
// 		v2.push_back(s2->get_x() + direc[i] * (car_wid * sin(s2->get_theta()) + car_len * cos(s2->get_theta())));
// 		v2.push_back(s2->get_y() + direc[i + 1] * (car_wid * cos(s2->get_theta()) + car_len * sin(s2->get_theta())));
// 	}

// 	v1.push_back(s1->get_x() + (car_wid * sin(s1->get_theta()) + car_len * cos(s1->get_theta())));
// 	v1.push_back(s1->get_y() + (car_wid * cos(s1->get_theta()) + car_len * sin(s1->get_theta())));
// 	v2.push_back(s2->get_x() + (car_wid * sin(s2->get_theta()) + car_len * cos(s2->get_theta())));
// 	v2.push_back(s2->get_y() + (car_wid * cos(s2->get_theta()) + car_len * sin(s2->get_theta())));

// 	for (int i = 0; i < 4; i++) {
// 		if (intersect_point(s1, v2[2 * i], v2[2 * i + 1])) {
// 		return true;
// 		}
// 	}
// 	printf("%lf, %lf, %lf, %lf\n", v1[0],v1[1],v1[2],v1[3]);
// 	printf("%lf, %lf, %lf, %lf\n", v2[0],v2[1],v2[2],v2[3]);
// 	for (int i = 0; i < 4; i++) {
// 		for (int j = 0; j < 4; j++) {
// 			if (intersect_lines(v1[2 * i], v1[2 * i + 1], v1[2 * (i + 1)], v1[2 * (i + 1) + 1], 
// 				v2[2 * i], v2[2 * i + 1], v2[2 * (i + 1)], v2[2 * (i + 1) + 1])) {
// 				return true;
// 			}
// 		}
// 	}
// 	return false;
// }
// Declear useful function header
bool total_collision_check(vector<State*> &obstacles, State *car);
#endif