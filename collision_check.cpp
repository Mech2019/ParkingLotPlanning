#include <math.h> 
#include <stdlib.h>

#define PI 3.14159265

double height = 4;
double width = 2;

bool collision_check(State const& s1, State const& s2){
	vector<int> direc = {1, 1, -1, -1, 1};

	for (int i = 0; i < 4; i++) {
		if (intersect_point(s1, s2.x + direc[i] * (w * sin(s2.theta) + h * cos(s2.theta)), 
		s2.y + direc[i + 1] * (w * cos(s2.theta) + h * sin(s2.theta)))) {
		return true;
		}
	}

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			if (intersect_lines(s2.x + direc[i] * (w * sin(s2.theta) + h * cos(s2.theta)), 
			s2.y + direc[i + 1] * (w * cos(s2.theta) + h * sin(s2.theta)), 
			s1.x + direc[j] * (w * sin(s1.theta) + h * cos(s1.theta)), 
			s1.y + direc[j + 1] * (w * cos(s1.theta) + h * sin(s1.theta)))) {
				return true;
			}
		}
	}
	return false;
}

bool intersect_point(State const& s1, double x, double y) {
	return (x > s1.x - height && x < s1.x + height && y < s1.y + width && y > s1.y + width);
}

bool onSegment(int x1, int y1, int x2, int y2, int x3, int y3) 
{ 
    if (x2 <= max(x1, x3) && x2 >= min(x1, x3) && 
        y2 <= max(y1, y3) && y2 >= min(y1, y3)) 
       return true; 
  
    return false; 
} 

int orientation(int x1, int y1, int x2, int y2, int x3, int y3) 
{ 
    int val = (y2 - y1) * (x3 - x2) - 
              (x2 - x1) * (y3 - y2); 
  
    if (val == 0) return 0;  // colinear 
  
    return (val > 0)? 1: 2; // clock or counterclock wise 
} 

bool intersect_lines(x1, y1, x2, y2, x3, y3, x4, y4) {
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