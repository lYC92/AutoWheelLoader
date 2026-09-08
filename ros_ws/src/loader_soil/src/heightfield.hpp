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
struct Surface {
  double height=0, gx=0, gy=0;
  double At(Point p) const { return height+gx*p[0]+gy*p[1]; }
};
struct Fragment { Polygon polygon; double height; double gx=0,gy=0;
  Surface Plane() const { return {height,gx,gy}; }
};
using Point3 = std::array<double,3>;

inline double Area(const Polygon &p) {
  if (p.size() < 3) return 0;
  double sum = 0;
  for (std::size_t i=0; i<p.size(); ++i) {
    const auto &a=p[i], &b=p[(i+1)%p.size()];
    sum += (a[0]-p[0][0])*(b[1]-p[0][1])-(b[0]-p[0][0])*(a[1]-p[0][1]);
  }
  return std::abs(sum)*0.5;
}
inline Polygon ConvexHull(Polygon points) {
  std::sort(points.begin(),points.end());
  points.erase(std::unique(points.begin(),points.end()),points.end());
  if(points.size()<3) return points;
  auto cross=[](Point a,Point b,Point c) {
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
  };
  Polygon hull;
  for(const auto &p:points) {
    while(hull.size()>1 && cross(hull[hull.size()-2],hull.back(),p)<=0) hull.pop_back();
    hull.push_back(p);
  }
  const auto lower=hull.size();
  for(auto it=points.rbegin()+1;it!=points.rend();++it) {
    while(hull.size()>lower && cross(hull[hull.size()-2],hull.back(),*it)<=0) hull.pop_back();
    hull.push_back(*it);
  }
  hull.pop_back();
  return hull;
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

// Signed linear clipping also represents intersection of two sloped surfaces.
inline std::pair<Polygon,Polygon> SplitSurface(const Polygon &p, Surface surface) {
  Polygon positive,negative;
  for(std::size_t i=0;i<p.size();++i) {
    const auto a=p[i],b=p[(i+1)%p.size()];
    const double da=surface.At(a),db=surface.At(b);
    if(da>=0) positive.push_back(a);
    if(da<=0) negative.push_back(a);
    if((da>0 && db<0)||(da<0 && db>0)) {
      const double t=da/(da-db);
      Point cross{a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])};
      positive.push_back(cross); negative.push_back(cross);
    }
  }
  return {positive,negative};
}
inline double Integral(const Polygon &p, Surface plane) {
  double volume=0;
  for(std::size_t i=1;i+1<p.size();++i)
    volume+=Area({p[0],p[i],p[i+1]})*(plane.At(p[0])+plane.At(p[i])+plane.At(p[i+1]))/3;
  return volume;
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
  std::size_t FragmentCount() const { std::size_t n=0; for(const auto &f:fragments) n+=f.size(); return n; }
  bool SubdivisionConsistent(double tolerance=1e-10) const {
    for(std::size_t i=0;i<fragments.size();++i) {
      if(fragments[i].empty()) continue;
      double area=0, volume=0;
      for(const auto &part:fragments[i]) {
        const double a=Area(part.polygon);
        area+=a; volume+=Integral(part.polygon,part.Plane());
      }
      if(std::abs(area-resolution*resolution)>tolerance ||
         std::abs(volume-heights[i]*resolution*resolution)>tolerance) return false;
    }
    return true;
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
    return TrianglePlane(std::move(triangle),{std::max(0.0,target),0,0});
  }
  double TrianglePlane(Polygon triangle, Surface target) {
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
        const double lowest=std::max(0.,std::min({target.At({x,y}),target.At({x+r,y}),target.At({x+r,y+r}),target.At({x,y+r})}));
        if(heights[i]<=lowest) continue; // No surface intersection; keep untouched cells implicit.
        fragments[i].push_back({{{x,y},{x+r,y},{x+r,y+r},{x,y+r}},heights[i]});
      }
      std::vector<Fragment> updated;
      for (const auto &f:fragments[i]) {
        if (payload>=capacity) { updated.push_back(f); continue; }
        const Surface plane=f.Plane();
        bool cuts=false;
        for(auto point:f.polygon) if(plane.At(point)>std::max(0.0,target.At(point))) {cuts=true;break;}
        if(!cuts) {updated.push_back(f);continue;}
        // Small physics substeps intersect only a few of a cell's retained
        // polygons. Reject disjoint bounding boxes before allocating clips.
        double fx0=f.polygon[0][0], fx1=fx0, fy0=f.polygon[0][1], fy1=fy0;
        for (const auto &point:f.polygon) {
          fx0=std::min(fx0,point[0]); fx1=std::max(fx1,point[0]);
          fy0=std::min(fy0,point[1]); fy1=std::max(fy1,point[1]);
        }
        if (fx1<=xmin || fx0>=xmax || fy1<=ymin || fy0>=ymax) {
          updated.push_back(f); continue;
        }
        Polygon inside=f.polygon;
        std::vector<Fragment> outside;
        for (int edge=0;edge<3;++edge) {
          auto parts=Split(inside,triangle[edge],triangle[(edge+1)%3]);
          inside=std::move(parts.first);
          if (Area(parts.second)>1e-16) outside.push_back({std::move(parts.second),f.height,f.gx,f.gy});
          if (inside.empty()) break;
        }
        double area=Area(inside);
        if (area<=1e-16) { updated.push_back(f); continue; }
        auto groundSplit=SplitSurface(inside,target);
        std::vector<Fragment> cutsToApply;
        double available=0;
        for(int piece=0;piece<2;++piece) {
          const auto &polygon=piece==0 ? groundSplit.first : groundSplit.second;
          if(Area(polygon)<=1e-16) continue;
          // When target is exactly zero, both clips include the same polygon.
          if(piece==1 && target.height==0 && target.gx==0 && target.gy==0) continue;
          const Surface cut=piece==0 ? target : Surface{};
          Surface difference{plane.height-cut.height,plane.gx-cut.gx,plane.gy-cut.gy};
          auto affected=SplitSurface(polygon,difference);
          const double v=Integral(affected.first,difference);
          if(v>1e-16) {
            available+=v;
            cutsToApply.push_back({std::move(affected.first),cut.height,cut.gx,cut.gy});
            if(Area(affected.second)>1e-16)
              outside.push_back({std::move(affected.second),f.height,f.gx,f.gy});
          } else outside.push_back({polygon,f.height,f.gx,f.gy});
        }
        if(available<=1e-16) {updated.push_back(f);continue;}
        const double actual=std::min(available,std::max(0.0,capacity-payload));
        const double fraction=actual/available;
        updated.insert(updated.end(),outside.begin(),outside.end());
        for(auto &cut:cutsToApply) {
          if(fraction<1) {
            cut.height=f.height+fraction*(cut.height-f.height);
            cut.gx=f.gx+fraction*(cut.gx-f.gx);
            cut.gy=f.gy+fraction*(cut.gy-f.gy);
          }
          updated.push_back(std::move(cut));
        }
        heights[i]-=actual/(resolution*resolution);
        payload+=actual; excavated+=actual; removed+=actual;
        if (actual>0) dirty[i]=true;
      }
      const bool subdivided=updated.size()>fragments[i].size();
      fragments[i]=std::move(updated);
      if (subdivided) Compact(i);
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
  double Sweep3D(Point3 a,Point3 b,Point3 c,Point3 d) {
    Point3 m0{},m1{};
    for(int i=0;i<3;++i) {m0[i]=(a[i]+b[i])/2;m1[i]=(c[i]+d[i])/2;}
    double volume=0;
    for(const auto &t:std::array<std::array<Point3,3>,4>{{{{a,m0,m1}},{{a,m1,d}},{{m0,b,c}},{{m0,c,m1}}}}) {
      const auto p=t[0],q=t[1],r=t[2];
      const double determinant=(q[0]-p[0])*(r[1]-p[1])-(r[0]-p[0])*(q[1]-p[1]);
      if(std::abs(determinant)<2e-16) continue;
      const double gx=((q[2]-p[2])*(r[1]-p[1])-(r[2]-p[2])*(q[1]-p[1]))/determinant;
      const double gy=((q[0]-p[0])*(r[2]-p[2])-(r[0]-p[0])*(q[2]-p[2]))/determinant;
      volume+=TrianglePlane({{p[0],p[1]},{q[0],q[1]},{r[0],r[1]}},{p[2]-gx*p[0]-gy*p[1],gx,gy});
    }
    return volume;
  }
  double Deposit(double x,double y,double requested,double slope) {
    if (Cell(x,y)<0 || requested<=0) return 0; // Keep payload outside the domain.
    requested=std::min(requested,payload);
    if(!std::isfinite(slope) || slope<=0) throw std::invalid_argument("invalid deposition slope");
    // Outside apex/slope the cone is below the nonnegative rigid ground;
    // those cells cannot receive material. Rebuild this exact support only
    // if the upper bracket grows, instead of visiting the whole yard 60 times.
    std::vector<std::pair<int,double>> support;
    auto bound=[&](double apex) {
      support.clear();const double radius=apex/slope;
      const int ix0=std::max(0,static_cast<int>(std::floor((x-radius-x0)/resolution)));
      const int ix1=std::min(nx-1,static_cast<int>(std::floor((x+radius-x0)/resolution)));
      const int iy0=std::max(0,static_cast<int>(std::floor((y-radius-y0)/resolution)));
      const int iy1=std::min(ny-1,static_cast<int>(std::floor((y+radius-y0)/resolution)));
      for(int iy=iy0;iy<=iy1;++iy) for(int ix=ix0;ix<=ix1;++ix) {
        const int i=iy*nx+ix;const double radial=slope*std::hypot(X(i)-x,Y(i)-y);
        if(radial<apex) support.emplace_back(i,radial);
      }
    };
    auto added=[&](double apex) {
      double sum=0;
      for(const auto &[i,radial]:support) sum+=std::max(0.0,apex-radial-heights[i]);
      return sum*resolution*resolution;
    };
    double low=0,high=*std::max_element(heights.begin(),heights.end())+1;
    bound(high);
    while(added(high)<requested) {high*=2;bound(high);}
    for(int k=0;k<60;++k) { double mid=(low+high)/2; if(added(mid)<requested) low=mid; else high=mid; }
    for(const auto &[i,radial]:support) {
      double next=std::max(heights[i],(low+high)/2-radial);
      if(next>heights[i]) { heights[i]=next; dirty[i]=true; fragments[i].clear(); }
    }
    payload-=requested; dumped+=requested;
    return requested;
  }
private:
  void Compact(int cell) {
    auto &parts=fragments[cell];
    if(parts.size()<16) return;
    // Combine equal-height neighbours only if their union is convex and its
    // area is preserved. Never average distinct heights to cap fragment count:
    // that would refill excavated subcells and allow double counting on retrace.
    for(std::size_t a=0;a<parts.size();++a) {
      for(std::size_t b=a+1;b<parts.size();) {
        if(parts[a].height!=parts[b].height || parts[a].gx!=parts[b].gx || parts[a].gy!=parts[b].gy) { ++b; continue; }
        Polygon vertices=parts[a].polygon;
        vertices.insert(vertices.end(),parts[b].polygon.begin(),parts[b].polygon.end());
        auto hull=ConvexHull(std::move(vertices));
        const double area=Area(parts[a].polygon)+Area(parts[b].polygon);
        if(std::abs(Area(hull)-area)<=1e-14*resolution*resolution) {
          parts[a].polygon=std::move(hull);
          parts.erase(parts.begin()+b);
          b=a+1;
        } else ++b;
      }
    }
  }
  std::vector<std::vector<Fragment>> fragments;
};
} // namespace loader_soil
