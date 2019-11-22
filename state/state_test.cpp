
#include "state.h"

#include <iostream>
using namespace std;


int main(){

  State ego_state = State(0, 0, 0, 0);
  State test_state = State(1, 1, 1, 0);

  cout << ego_state.get_x() << endl;
  cout << test_state.get_x() << endl;

  ego_state.print_state();
  test_state.print_state();

  return 0;
}