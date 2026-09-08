#include "../../ros_ws/src/loader_soil/src/heightfield.hpp"
#include <chrono>
#include <iostream>
#include <random>
#include <string>

using namespace loader_soil;
void require(bool ok, const char *message) {
  if (!ok) throw std::runtime_error(message);
}
void conserved(const Heightfield &h) {
  require(std::abs(h.Volume()+h.payload-h.initial)<1e-9, "material balance changed");
  require(h.payload>=-1e-12 && h.payload<=h.capacity+1e-12, "payload outside capacity");
  require(h.SubdivisionConsistent(), "subcell areas or integrals disagree with terrain ledger");
  for(double height:h.heights) require(std::isfinite(height) && height>=-1e-12, "invalid terrain height");
}
Heightfield flat(double capacity) {
  Heightfield h(-2,-2,32,32,0.125,capacity);
  std::fill(h.heights.begin(),h.heights.end(),1);
  h.initial=h.Volume();
  return h;
}
void stroke(Heightfield &h, double yaw, double z) {
  auto p=[&](double x,double y) {return Point{x*std::cos(yaw)-y*std::sin(yaw),x*std::sin(yaw)+y*std::cos(yaw)};};
  for(int k=0;k<100;++k) {
    const double x=-0.5+k*0.01;
    h.Sweep(p(x,-0.4),p(x,0.4),p(x+0.01,0.4),p(x+0.01,-0.4),z);
  }
}
int main() {
  const auto started=std::chrono::steady_clock::now();
  // A full bucket must leave every terrain cell unchanged on further cuts.
  auto full=flat(0.137);
  stroke(full,0.31,0.1);
  require(std::abs(full.payload-full.capacity)<1e-12,"bucket did not reach capacity");
  auto snapshot=full.heights;
  auto count=full.FragmentCount();
  for(int i=0;i<100;++i) stroke(full,i*0.013,0);
  require(full.heights==snapshot && full.FragmentCount()==count,"full bucket still excavated");
  conserved(full);
  for(Point p: {Point{-2.01,0},Point{2,0},Point{0,-2.01},Point{0,2}}) {
    require(full.Deposit(p[0],p[1],full.payload,0.67)==0,"outside deposit accepted");
    require(full.heights==snapshot && std::abs(full.payload-0.137)<1e-12,"outside deposit lost material");
  }
  require(std::abs(full.Deposit(-2,-2,1,0.67)-0.137)<1e-12,"edge deposit lost material");
  conserved(full);

  std::cerr << "capacity/boundary passed\n";
  auto repeat=flat(100);
  stroke(repeat,0.413,0.3);
  require(std::abs(repeat.payload-0.56)<1e-9,"rotated analytical cut volume wrong");
  const double captured=repeat.payload;
  count=repeat.FragmentCount();
  for(int i=0;i<1000;++i) { stroke(repeat,0.413,0.3); if(i%100==0) std::cerr << "retrace " << i << " fragments=" << repeat.FragmentCount() << "\n"; }
  require(std::abs(repeat.payload-captured)<1e-10,"retrace excavated material twice");
  require(repeat.FragmentCount()==count,"identical retrace grew subcell fragments");
  conserved(repeat);

  // Persistent terrain: recycle excavated material, never reset the grid or
  // its ledger between cycles. Deterministic varied headings and dump points.
  auto stress=flat(0.08);
  std::mt19937 random(20260908);
  std::uniform_real_distribution<double> unit(0,1);
  std::size_t peak=0;
  int productive=0;
  for(int i=0;i<1000;++i) {
    stroke(stress,unit(random)*6.283185307179586,0.05+0.7*unit(random));
    if(stress.payload>1e-8) ++productive;
    stress.Deposit(-1.9+3.8*unit(random),-1.9+3.8*unit(random),stress.payload,0.67);
    conserved(stress);
    peak=std::max(peak,stress.FragmentCount());
    if(i%100==0) std::cerr << "persistent " << i << " fragments=" << stress.FragmentCount() << "\n";
  }
  require(productive>=100,"stress sequence stopped exercising material transfer");
  require(peak<10000,"nominal stress exceeded fragment budget");
  const double seconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();
  std::cout << "{\"status\":\"passed\",\"retrace_passes\":1000,\"retrace_fragments\":" << count
            << ",\"persistent_cycles\":1000,\"productive_cycles\":" << productive
            << ",\"peak_fragments\":" << peak << ",\"final_fragments\":" << stress.FragmentCount()
            << ",\"balance_m3\":" << stress.Volume()+stress.payload-stress.initial
            << ",\"wall_seconds\":" << seconds << "}\n";
}
