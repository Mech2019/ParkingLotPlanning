
class State{
private:
  double x;
  double y;
  double theta;
  bool flag; //ego_vehicle: forward/backward; map: empty/full
public:
  State(){
    x = 0;
    y = 0;
    theta = 0;
    flag = 0;
  }

  State(double x_, double y_, double theta_, bool flag_){
    x = x_;
    y = y_;
    theta = theta_;
    flag = flag_; // default forward
  }

  void update_state(int x_, int y_, double theta_, bool flag_){
    x = x_;
    y = y_;
    theta = theta_;
    flag = flag_;
  }

  double get_x(){
    return x;
  }

  double get_y(){
    return y;
  }

  double get_theta(){
    return theta;
  }

  bool get_flag(){
    return flag;
  }

  void set_x(double x_){
    x = x_;
  }

  void set_y(double y_){
    y = y_;
  }

  void set_theta(double theta_){
    theta = theta_;
  }

};





