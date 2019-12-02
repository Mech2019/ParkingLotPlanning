#ifndef STATE_H
#define STATE_H

#include <vector>
#include <iostream>
#include <functional>
#include <fstream>

using namespace std;

class State{
protected:	// changed private to protected for inheritance convenience
	double x;
	double y;
	double theta;
	bool flag; // ego_vehicle: forward/backward; map: empty/full

public:
	//constructor
	State();
	State(double x_, double y_, double theta_, bool flag_);

	void update_state(int x_, int y_, double theta_, bool flag_);
	double get_x();
	double get_y();
	double get_theta();
	bool get_flag();
	void set_x(double x_);
	void set_y(double y_);
	void set_theta(double theta_);

	bool operator==(State *rhs) const;
	friend std::ostream& operator<<(std::ostream& os, State *s)
    {
	    os << s->get_x() << " " << s->get_y() << " " 
	    	<< s->get_theta() << " " << s->get_flag() << " ";
	    return os;
	}
};

/* Added comparator and hasher for the State class for future container use */
struct StateComparator{
	bool operator()(State *lhs, State *rhs) const{
		return lhs == rhs;
	}
};

struct StateHasher{
	size_t operator()(State *state) const{
		return std::hash<double>{}(state->get_x());
	}
};

/*****************************************************************************/
class CarState : public State {
private:
	double delta; // steering angle
	CarState nextCarState(CarState car, double dt) const;

public:
	CarState();
	CarState(double x_, double y_, double theta_, bool flag_, double delta_);

	friend ostream& operator<<(ostream& os, CarState& car);

	double get_delta();
	void set_delta(double delta_);

	// input: 
	//	empty 2D vector of CarState to store sampled locations
	// Output: 
	//	filled 2D vector
	//	Each row in this vector represents the trajectoy of 1 primitive
	//	with the last element to be the final location after performing that primitive
	void compute_primitive(std::vector<std::vector<CarState> > &result, vector<State*> & obstacles) const;
};

#endif