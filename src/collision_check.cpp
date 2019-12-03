#include "util.h"

using namespace std; 

/* the original h, w are declared as static const variable in map.h */
// bool intersect_point(State *s1, double x, double y) {
// 	return (x >= s1->get_x() - car_wid/2 && x <= s1->get_x() + car_wid/2 
// 		&& y <= s1->get_y() + car_len/2 && y >= s1->get_y() - car_len/2);
// }
bool intersect_point(State *s1, double x, double y) {
    double new_x = x - s1->get_x();
    double new_y = y - s1->get_y();
    double proj_x = new_x * cos(s1->get_theta()) + new_y * sin(s1->get_theta());
    double proj_y = new_x * sin(s1->get_theta()) + new_y * cos(s1->get_theta());
    return (abs(proj_x) <= car_len/2 && abs(proj_y) <= car_wid/2);
}

bool onSegment(double x1, double y1, double x2, double y2, double x3, double y3) 
{ 
    if (x2 <= MAX(x1, x3) && x2 >= MIN(x1, x3) && 
        y2 <= MAX(y1, y3) && y2 >= MIN(y1, y3)) 
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

template <class T>
bool collision_check(State *s1, T *s2){
	int direc[] = {1, 1, -1, -1, 1}; 

	std::vector<double> v1, v2;
	for (int i = 0; i < 4; i++) {
		v1.push_back(s1->get_x() + direc[i] * (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
		v1.push_back(s1->get_y() + direc[i + 1] * (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
		v2.push_back(s2->get_x() + direc[i] * (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
		v2.push_back(s2->get_y() + direc[i + 1] * (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));
	}

	v1.push_back(s1->get_x() + (car_len/2 * sin(s1->get_theta()) + car_wid/2 * cos(s1->get_theta())));
	v1.push_back(s1->get_y() + (car_len/2 * cos(s1->get_theta()) + car_wid/2 * sin(s1->get_theta())));
	v2.push_back(s2->get_x() + (car_len/2 * sin(s2->get_theta()) + car_wid/2 * cos(s2->get_theta())));
	v2.push_back(s2->get_y() + (car_len/2 * cos(s2->get_theta()) + car_wid/2 * sin(s2->get_theta())));

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

bool total_collision_check(vector<State*> &obstacles, State *car) {
	bool result = false;
	for (State *s2 : obstacles) {
		result |= collision_check(s2, car);
        // printf("car state: %lf, %lf, %lf \n", car->get_x(), car->get_y(), car->get_theta());
        // printf("obs state: %lf, %lf, %lf \n", s2->get_x(), s2->get_y(), s2->get_theta());
		if (result) return result;
	}
	return result;
}
