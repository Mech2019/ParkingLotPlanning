#define CAR_SPEED 5.0
#define PI 3.1415927

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






