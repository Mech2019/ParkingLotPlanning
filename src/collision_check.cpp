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



// bool linesegmentcheck(double x1, double x2, double x3, double x4,
// 								double y1, double y2, double y3, double y4){
// 	if (x1 == x2 && x3 == x4){
// 		// printf("case 1\n");
// 		// printf("x1==x3? %d\n", x1==x3);
// 		if (x1 == x3){
// 			// printf("im here\n");
// 			// printf("y3: %lf \n", (y3-y1)*(y3-y2));
// 			// printf("y4: %lf \n", (y4-y1)*(y4-y2));
// 			if ((y3-y1)*(y3-y2) <= 0 || (y4-y1)*(y4-y2) <= 0){
// 				return true;
// 			}
// 		}
// 	} else if (x1 == x2 && x3 != x4) {
// 		// printf("case 2\n");
// 		double m2 = (y4 - y3) / (x4 - x3);
// 		double b2 = y3 - m2 * x3;

// 		double x_intersect = x1;
// 		double y_intersect = m2 * x_intersect + b2;
// 		if ((x_intersect-x1)*(x_intersect-x2) <= 0
// 			&& (x_intersect-x3)*(x_intersect-x4) <= 0
// 			&& (y_intersect-y1)*(y_intersect-y2) <= 0
// 			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
// 			return true;
// 		}
// 	} else if (x1 != x2 && x3 == x4) {
// 		// printf("case 3\n");
// 		double m1 = (y2 - y1) / (x2 - x1);
// 		double b1 = y1 - m1 * x1;

// 		double x_intersect = x3;
// 		double y_intersect = m1 * x_intersect + b1;
// 		if ((x_intersect-x1)*(x_intersect-x2) <= 0
// 			&& (x_intersect-x3)*(x_intersect-x4) <= 0
// 			&& (y_intersect-y1)*(y_intersect-y2) <= 0
// 			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
// 			return true;
// 		}
// 		// printf("case3, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
// 	} else {
// 		// calculate slope and y-intersection for line
// 		double m1 = (y2 - y1) / (x2 - x1);
// 		double b1 = y1 - m1 * x1;
// 		double m2 = (y4 - y3) / (x4 - x3);
// 		double b2 = y3 - m2 * x3;
// 		// printf("line 1: m1 = %lf, b1 = %lf\n", m1, b1);
// 		// printf("line 2: m2 = %lf, b2 = %lf\n", m2, b2);
// 		// calculate intersection coordinate
// 		double x_intersect = (b2-b1)/(m1-m2);
// 		double y_intersect = m1 * x_intersect + b1;
// 		// check if the intersection point is on both segment
// 		if ((x_intersect-x1)*(x_intersect-x2) <= 0
// 			&& (x_intersect-x3)*(x_intersect-x4) <= 0
// 			&& (y_intersect-y1)*(y_intersect-y2) <= 0
// 			&& (y_intersect-y3)*(y_intersect-y4) <= 0){
// 			return true;
// 		}
// 		// printf("case4, intersect at (%lf, %lf)\n", x_intersect, y_intersect);
// 	}
// 	// printf("im here 2\n");
// 	return false;
// }