#include "../../ros_ws/src/loader_soil/src/heightfield.hpp"
#include <iostream>
using namespace loader_soil;
void check(bool ok) { if(!ok) throw std::runtime_error("heightfield regression failed"); }
int main() {
  for(double yaw : {0.0, 0.7853981633974483, 1.5707963267948966}) {
    Heightfield h(-2,-2,40,40,0.1,100);
    std::fill(h.heights.begin(),h.heights.end(),1.0); h.initial=h.Volume();
    auto point=[&](double x,double y) { return Point{x*std::cos(yaw)-y*std::sin(yaw),x*std::sin(yaw)+y*std::cos(yaw)}; };
    for(int pass=0;pass<2;++pass) for(int k=0;k<100;++k) {
      double a=-0.5+k*0.01,b=a+0.01;
      if(pass) std::swap(a,b);
      h.Sweep(point(a,-0.5),point(a,0.5),point(b,0.5),point(b,-0.5),0.5);
    }
    check(std::abs(h.payload-0.5)<1e-9);
    check(h.Deposit(8,8,0.5,0.67)==0);
    check(std::abs(h.Deposit(1,1,0.5,0.67)-0.5)<1e-9);
    check(std::abs(h.Volume()+h.payload-h.initial)<1e-9);
  }
  std::cout << "PASS C++ oriented sweep, retrace, XY deposit, boundary and conservation\n";
}
