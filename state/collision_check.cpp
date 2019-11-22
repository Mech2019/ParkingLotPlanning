#include <math.h> 
#include <stdlib.h>
#include <iostream>
#include <vector>
#include "planner.h"

using namespace std; 

double h = 4;
double w = 2;

bool intersect_point(State &s1, double x, double y) {
	return (x > s1.get_x() - h && x < s1.get_x() + h && y < s1.get_y() + w && y > s1.get_y() + w);
}

bool onSegment(double x1, double y1, double x2, double y2, double x3, double y3) 
{ 
    if (x2 <= max(x1, x3) && x2 >= min(x1, x3) && 
        y2 <= max(y1, y3) && y2 >= min(y1, y3)) 
       return true; 
  
    return false; 
} 

int orientation(double x1, double y1, double x2, double y2, double x3, double y3) 
{ 
    double val = (y2 - y1) * (x3 - x2) - 
              (x2 - x1) * (y3 - y2); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool intersect_lines(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
	int o1 = orientation(x1, y1, x2, y2, x3, y3); 
    int o2 = orientation(x1, y1, x2, y2, x4, y4); 
    int o3 = orientation(x3, y3, x4, y4, x1, y1); 
    int o4 = orientation(x3, y3, x4, y4, x2, y2); 
  
    // General case 
    if (o1 != o2 && o3 != o4) 
        return true; 
  
    // Special Cases 
    if (o1 == 0 && onSegment(x1, y1, x3, y3, x2, y2)) return true; 
  
    if (o2 == 0 && onSegment(x1, y1, x4, y4, x2, y2)) return true; 
  
    if (o3 == 0 && onSegment(x3, y3, x1, y1, x4, y4)) return true; 
  
    if (o4 == 0 && onSegment(x3, y3, x2, y2, x4, y4)) return true; 
  
    return false; 
}

bool collision_check(State &s1, State &s2){
	int direc[] = {1, 1, -1, -1, 1}; 

	std::vector<double> v1, v2;
	for (int i = 0; i < 4; i++) {
		v1.push_back(s1.get_x() + direc[i] * (w * sin(s1.get_theta()) + h * cos(s1.get_theta())));
		v1.push_back(s1.get_y() + direc[i + 1] * (w * cos(s1.get_theta()) + h * sin(s1.get_theta())));
		v2.push_back(s2.get_x() + direc[i] * (w * sin(s2.get_theta()) + h * cos(s2.get_theta())));
		v2.push_back(s2.get_y() + direc[i + 1] * (w * cos(s2.get_theta()) + h * sin(s2.get_theta())));
	}

	v1.push_back(s1.get_x() + (w * sin(s1.get_theta()) + h * cos(s1.get_theta())));
	v1.push_back(s1.get_y() + (w * cos(s1.get_theta()) + h * sin(s1.get_theta())));
	v2.push_back(s2.get_x() + (w * sin(s2.get_theta()) + h * cos(s2.get_theta())));
	v2.push_back(s2.get_y() + (w * cos(s2.get_theta()) + h * sin(s2.get_theta())));

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

int main(){
  printf("Finished !\n");

  State state1 = State(0, 0, 0, 0);
  State state2 = State(8, 0, 0, 0);
  cout << collision_check(state1, state2) << endl;
  printf("Finished !\n");

  return 0;
}