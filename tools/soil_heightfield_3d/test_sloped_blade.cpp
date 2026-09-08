#include "../../ros_ws/src/loader_soil/src/heightfield.hpp"
#include <iostream>
using namespace loader_soil;
void check(bool ok,const char *message) {if(!ok) throw std::runtime_error(message);}
Heightfield flat(double capacity=100) {
  Heightfield h(-1,-1,20,20,0.1,capacity);
  std::fill(h.heights.begin(),h.heights.end(),1);h.initial=h.Volume();return h;
}
void cut(Heightfield &h,double roll,double pitch=0) {
  for(int k=0;k<100;++k) {
    const double x=-0.5+k*0.01,q=x+0.01;
    h.Sweep3D({x,-1,0.5-roll+pitch*x},{x,1,0.5+roll+pitch*x},
              {q,1,0.5+roll+pitch*q},{q,-1,0.5-roll+pitch*q});
  }
}
int main() {
  for(double roll:{-0.4,0.0,0.4}) {
    auto h=flat();cut(h,roll,0.2);
    check(std::abs(h.payload-1)<1e-9,"sloping-plane analytical volume");
    for(int i=0;i<h.nx*h.ny;++i) if(std::abs(h.X(i))<0.49)
      check(std::abs(h.heights[i]-(0.5+roll*h.Y(i)+0.2*h.X(i)))<1e-9,"local cut elevation");
    cut(h,roll,0.2);
    check(std::abs(h.payload-1)<1e-9,"sloping-plane retrace");
    check(h.SubdivisionConsistent(),"sloping-plane subcell integration");
  }
  auto clamped=flat();cut(clamped,1);
  check(std::abs(clamped.payload-1)<1e-9,"ground and old-surface clipping");
  check(clamped.SubdivisionConsistent(),"clipped subdivision");
  auto crossed=flat();cut(crossed,0.4);cut(crossed,-0.4);
  check(std::abs(crossed.payload-1.4)<1e-9,"opposing slopes lower envelope");
  crossed.Sweep({-0.5,-1},{-0.5,1},{0.5,1},{0.5,-1},0.4);
  check(std::abs(crossed.payload-1.425)<1e-9,"flat cut through retained sloped fragments");
  check(crossed.SubdivisionConsistent(),"crossed surface subdivision");
  auto limited=flat(0.123);cut(limited,0.4);
  check(std::abs(limited.payload-0.123)<1e-12,"capacity on sloped cut");
  check(limited.SubdivisionConsistent(),"capacity interpolation of sloped surfaces");
  check(std::abs(limited.Volume()+limited.payload-limited.initial)<1e-9,"sloped conservation");
  std::cout << "PASS sloped blade: roll, pitch, local height, retrace, crossing, ground clip and capacity\n";
}
