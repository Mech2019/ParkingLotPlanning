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

bool intersect_point(State &s1, double x, double y);
bool onSegment(double x1, double y1, double x2, double y2, double x3, double y3);
int orientation(double x1, double y1, double x2, double y2, double x3, double y3);
bool intersect_lines(double x1, double y1, double x2, double y2, 
	double x3, double y3, double x4, double y4);
bool collision_check(State &s1, State &s2);

#endif