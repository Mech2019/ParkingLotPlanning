#include <iostream>
#include <vector>

using namespace std;

#define CAR_SPEED 5.0
#define PI 3.1415927

static const char *map_name = "map.csv";

class State{

private:
    double x;
    double y;
    double theta;
    bool flag; //ego_vehicle: forward/backward; map: empty/full

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
};

class CarState : public State {
private:
	double delta; // steering angle
	CarState nextCarState(CarState car, double dt) const;

public:
	CarState();
	CarState(double x_, double y_, double theta_, bool flag_, double delta_);

	double get_delta();
	void set_delta(double delta_);

	void compute_primitive(vector<vector<CarState>> &result) const;
};

/*****      map     *****/
class static_map{
private:
    double map_width;
    double map_length;
    int slot_num;
    vector<State*> slots;
public:
    static_map(){}
    static_map(double wid, double len, vector<vector<double>> input);
    ~static_map();
    int get_slot_num();
    double get_map_width();
    double get_map_length();
    vector<State*> get_slots();
};

void parse_static_map(char* filename, vector<vector<double>>& input);



