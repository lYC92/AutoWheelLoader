#include "payload_body.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <iostream>
#include <map>

namespace loader_soil {
class PayloadProbe : public gz::sim::System,
 public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
 public gz::sim::ISystemPostUpdate {
  gz::sim::Link bucket;
  gz::sim::Entity world;
  std::unique_ptr<gz::sim::SdfEntityCreator> creator;
  PayloadBody payload;
  std::map<int,std::pair<double,double>> samples;
  bool reported=false;
public:
  void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element>&,
                 gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &events) override {
    bucket=gz::sim::Link(gz::sim::Model(entity).LinkByName(ecm,"bucket"));
    bucket.EnableVelocityChecks(ecm);world=ecm.ParentEntity(entity);
    creator=std::make_unique<gz::sim::SdfEntityCreator>(ecm,events);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override {
    if(info.paused) return;
    const double time=std::chrono::duration<double>(info.simTime).count();
    const auto pose=bucket.WorldPose(ecm);if(!pose) return;
    const bool loaded=time>=1 && time<3;
    payload.Update(loaded?0.5:0,1000,time,bucket,*pose,ecm.ParentEntity(ecm.ParentEntity(bucket.Entity())),ecm,*creator);
    const auto center=loaded ? Payload(0.5,1000).center/3 : gz::math::Vector3d::Zero;
    if(time<2 || time>=3) bucket.AddWorldForce(ecm,{1500,0,0},center);
    else bucket.AddWorldWrench(ecm,{0,0,0},{0,123,0});
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    const double time=std::chrono::duration<double>(info.simTime).count();
    const int phase=static_cast<int>(time),tick=static_cast<int>(std::round(time*1000));
    if(phase>3) return;
    const bool loaded=phase==1 || phase==2;
    const auto center=loaded ? Payload(0.5,1000).center/3 : gz::math::Vector3d::Zero;
    const auto linear=bucket.WorldLinearVelocity(ecm,center),angular=bucket.WorldAngularVelocity(ecm);
    if(!linear || !angular) return;
    if(tick%1000==200 || tick%1000==800)
      samples[tick]={linear->X(),angular->Y()};
    if(time<3.81 || reported) return;
    reported=true;
    auto acceleration=[&](int start,bool rotation=false) {
      return ((rotation?samples.at(start+600).second:samples.at(start+600).first)-
              (rotation?samples.at(start).second:samples.at(start).first))/0.6;
    };
    const auto p=Payload(0.5,1000);
    const double inertia=1000+p.diagonal.Y()+1000.0/3*(p.center.X()*p.center.X()+p.center.Z()*p.center.Z());
    const double empty=acceleration(200),loadedA=acceleration(1200),unloaded=acceleration(3200),alpha=acceleration(2200,true);
    const bool passed=std::abs(empty-1.5)<0.02 && std::abs(loadedA-1)<0.02 &&
                      std::abs(unloaded-1.5)<0.02 && std::abs(alpha-123/inertia)<0.005;
    std::cout << (passed?"PASS":"FAIL") << " payload dynamics: empty_ax=" << empty
              << " loaded_ax=" << loadedA << " unloaded_ax=" << unloaded
              << " angular_ay=" << alpha << " expected_ay=" << 123/inertia << std::endl;
  }
};
}
GZ_ADD_PLUGIN(loader_soil::PayloadProbe,gz::sim::System,
 loader_soil::PayloadProbe::ISystemConfigure,loader_soil::PayloadProbe::ISystemPreUpdate,
 loader_soil::PayloadProbe::ISystemPostUpdate)
