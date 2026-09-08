#include "../../ros_ws/src/loader_soil/src/heightfield.hpp"
#include <chrono>
#include <iostream>
using namespace loader_soil;

void dense(Heightfield &h,double x,double y,double volume,double slope) {
  std::vector<double> radial(h.heights.size());
  for(int i=0;i<h.nx*h.ny;++i)radial[i]=slope*std::hypot(h.X(i)-x,h.Y(i)-y);
  auto added=[&](double apex){double sum=0;for(int i=0;i<h.nx*h.ny;++i)sum+=std::max(0.,apex-radial[i]-h.heights[i]);return sum*h.resolution*h.resolution;};
  double lo=0,hi=*std::max_element(h.heights.begin(),h.heights.end())+1;
  while(added(hi)<volume)hi*=2;
  for(int j=0;j<60;++j){double mid=(lo+hi)/2;if(added(mid)<volume)lo=mid;else hi=mid;}
  for(int i=0;i<h.nx*h.ny;++i)h.heights[i]=std::max(h.heights[i],(lo+hi)/2-radial[i]);
}
int main(){
  double fastSeconds=0,denseSeconds=0;
  for(auto p:std::array<Point,4>{{{4,17.8},{-14,-6},{17.999,23.999},{7,0}}}) {
    Heightfield fast(-14,-6,256,240,.125,20);fast.Pile(7,0,1.8,.67);auto slow=fast;
    for(double volume:{.006,.2,8.}) {
      fast.payload=volume;
      auto start=std::chrono::steady_clock::now();fast.Deposit(p[0],p[1],volume,.67);
      fastSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      start=std::chrono::steady_clock::now();dense(slow,p[0],p[1],volume,.67);
      denseSeconds+=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
      for(std::size_t i=0;i<fast.heights.size();++i)
        if(std::abs(fast.heights[i]-slow.heights[i])>1e-12)throw std::runtime_error("bounded deposit disagrees with dense reference");
    }
  }
  std::cout<<"PASS bounded deposition equals dense reference: optimized="<<fastSeconds<<" s, dense="<<denseSeconds<<" s\n";
}
