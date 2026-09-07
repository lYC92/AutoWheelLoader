#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace loader_soil {
using Point = std::array<double, 2>;
using Polygon = std::vector<Point>;
struct Fragment { Polygon polygon; double height; };

inline double Area(const Polygon &p) {
  if (p.size() < 3) return 0;
  double sum = 0;
  for (std::size_t i=0; i<p.size(); ++i) {
    const auto &a=p[i], &b=p[(i+1)%p.size()];
    sum += (a[0]-p[0][0])*(b[1]-p[0][1])-(b[0]-p[0][0])*(a[1]-p[0][1]);
  }
  return std::abs(sum)*0.5;
}
inline std::pair<Polygon, Polygon> Split(const Polygon &p, Point a, Point b) {
  Polygon in, out;
  auto side=[&](Point q) { return (b[0]-a[0])*(q[1]-a[1])-(b[1]-a[1])*(q[0]-a[0]); };
  for (std::size_t i=0; i<p.size(); ++i) {
    auto q=p[i], r=p[(i+1)%p.size()];
    double dq=side(q), dr=side(r);
    if (dq>=0) in.push_back(q);
    if (dq<=0) out.push_back(q);
    if ((dq>0 && dr<0) || (dq<0 && dr>0)) {
      double t=dq/(dq-dr);
      Point cross{q[0]+t*(r[0]-q[0]), q[1]+t*(r[1]-q[1])};
      in.push_back(cross); out.push_back(cross);
    }
  }
  return {in,out};
}

// Conservative XY grid. Subcell polygons are retained until that cell receives
// deposited material; the public height is the volume-equivalent cell average.
class Heightfield {
public:
  double x0, y0, resolution, capacity, payload=0, initial=0, excavated=0, dumped=0;
  int nx, ny;
  std::vector<double> heights;
  std::vector<bool> dirty;
  Heightfield(double x, double y, int cols, int rows, double r, double cap)
      : x0(x), y0(y), resolution(r), capacity(cap), nx(cols), ny(rows) {
    if (cols<=0 || rows<=0 || !std::isfinite(r) || r<=0 || cap<=0)
      throw std::invalid_argument("invalid heightfield dimensions");
    heights.assign(nx*ny,0); dirty.assign(nx*ny,true); fragments.resize(nx*ny);
  }
  double X(int i) const { return x0+(i%nx+0.5)*resolution; }
  double Y(int i) const { return y0+(i/nx+0.5)*resolution; }
  double Volume() const { return std::accumulate(heights.begin(),heights.end(),0.0)*resolution*resolution; }
  int Cell(double x, double y) const {
    int ix=static_cast<int>(std::floor((x-x0)/resolution));
    int iy=static_cast<int>(std::floor((y-y0)/resolution));
    return ix>=0 && ix<nx && iy>=0 && iy<ny ? iy*nx+ix : -1;
  }
  void Pile(double x, double y, double h, double slope) {
    for (int i=0;i<nx*ny;++i) heights[i]=std::max(0.0,h-slope*std::hypot(X(i)-x,Y(i)-y));
    initial=Volume();
  }
  double Triangle(Polygon triangle, double target) {
    target=std::max(0.0,target);
    if (Area(triangle)<1e-16 || payload>=capacity) return 0;
    auto a=triangle[0], b=triangle[1], c=triangle[2];
    if ((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])<0)
      std::reverse(triangle.begin(),triangle.end());
    double xmin=std::min({a[0],b[0],c[0]}), xmax=std::max({a[0],b[0],c[0]});
    double ymin=std::min({a[1],b[1],c[1]}), ymax=std::max({a[1],b[1],c[1]});
    int ix0=std::max(0,static_cast<int>(std::floor((xmin-x0)/resolution)));
    int ix1=std::min(nx-1,static_cast<int>(std::floor((xmax-x0)/resolution)));
    int iy0=std::max(0,static_cast<int>(std::floor((ymin-y0)/resolution)));
    int iy1=std::min(ny-1,static_cast<int>(std::floor((ymax-y0)/resolution)));
    double removed=0;
    for(int iy=iy0;iy<=iy1;++iy) for(int ix=ix0;ix<=ix1;++ix) {
      int i=iy*nx+ix;
      if (payload>=capacity-1e-14) return removed;
      if (fragments[i].empty()) {
        double x=x0+ix*resolution,y=y0+iy*resolution,r=resolution;
        fragments[i].push_back({{{x,y},{x+r,y},{x+r,y+r},{x,y+r}},heights[i]});
      }
      std::vector<Fragment> updated;
      for (const auto &f:fragments[i]) {
        if (f.height<=target) { updated.push_back(f); continue; }
        Polygon inside=f.polygon;
        std::vector<Fragment> outside;
        for (int edge=0;edge<3;++edge) {
          auto parts=Split(inside,triangle[edge],triangle[(edge+1)%3]);
          inside=std::move(parts.first);
          if (Area(parts.second)>1e-16) outside.push_back({std::move(parts.second),f.height});
          if (inside.empty()) break;
        }
        double area=Area(inside);
        if (area<=1e-16) { updated.push_back(f); continue; }
        double actual=std::min(area*(f.height-target),std::max(0.0,capacity-payload));
        updated.insert(updated.end(),outside.begin(),outside.end());
        updated.push_back({std::move(inside),f.height-actual/area});
        heights[i]-=actual/(resolution*resolution);
        payload+=actual; excavated+=actual; removed+=actual;
        if (actual>0) dirty[i]=true;
      }
      fragments[i]=std::move(updated);
    }
    return removed;
  }
  // Half-blade subdivision also covers pure rotation without bow-tie polygons.
  double Sweep(Point a, Point b, Point c, Point d, double z) {
    Point m0{(a[0]+b[0])/2,(a[1]+b[1])/2}, m1{(c[0]+d[0])/2,(c[1]+d[1])/2};
    double volume=0;
    for (const auto &p:std::vector<Polygon>{{a,m0,m1},{a,m1,d},{m0,b,c},{m0,c,m1}})
      volume+=Triangle(p,z);
    return volume;
  }
  double Deposit(double x,double y,double requested,double slope) {
    if (Cell(x,y)<0 || requested<=0) return 0; // Keep payload outside the domain.
    requested=std::min(requested,payload);
    std::vector<double> radial(heights.size());
    for(int i=0;i<nx*ny;++i) radial[i]=slope*std::hypot(X(i)-x,Y(i)-y);
    auto added=[&](double apex) {
      double sum=0;
      for(int i=0;i<nx*ny;++i) sum+=std::max(0.0,apex-radial[i]-heights[i]);
      return sum*resolution*resolution;
    };
    double low=0,high=*std::max_element(heights.begin(),heights.end())+1;
    while(added(high)<requested) high*=2;
    for(int k=0;k<60;++k) { double mid=(low+high)/2; if(added(mid)<requested) low=mid; else high=mid; }
    for(int i=0;i<nx*ny;++i) {
      double next=std::max(heights[i],(low+high)/2-radial[i]);
      if(next>heights[i]) { heights[i]=next; dirty[i]=true; fragments[i].clear(); }
    }
    payload-=requested; dumped+=requested;
    return requested;
  }
private:
  std::vector<std::vector<Fragment>> fragments;
};
} // namespace loader_soil
