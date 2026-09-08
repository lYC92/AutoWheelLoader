#pragma once
#include <cmath>
#include <gz/math/Inertial.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>

namespace loader_soil {
struct PayloadProperties {
  double mass=0;
  gz::math::Vector3d center, diagonal;
};
inline PayloadProperties Payload(double volume,double density) {
  if(!std::isfinite(volume) || !std::isfinite(density) || volume<0 || density<=0)
    throw std::invalid_argument("invalid payload volume/density");
  constexpr double length=1.2,width=2.3;
  const double height=volume/(length*width),mass=volume*density;
  return {mass,{0.35,0,-0.5+height/2},
    {mass*(width*width+height*height)/12,
     mass*(length*length+height*height)/12,
     mass*(length*length+width*width)/12}};
}

// Harmonic's stock Physics system does not apply edits of an existing link's
// Inertial component to DART. A replaceable, rigidly attached payload body puts
// the actual mass/COM/inertia into the solver. Updates are discrete at 20 Hz;
// nominal added mass is never also applied as an external gravity wrench.
class PayloadBody {
public:
  PayloadProperties properties;
  double volume=0;
  void Update(double desired,double density,double simTime,
              gz::sim::Link bucket,const gz::math::Pose3d &pose,
              gz::sim::Entity world,gz::sim::EntityComponentManager &ecm,
              gz::sim::SdfEntityCreator &creator) {
    // Detach in one iteration, remove the freed model in the next. Removing
    // its source skeleton before DART has detached the body invalidates joints.
    if(retiringModel!=gz::sim::kNullEntity) {
      ecm.RequestRemoveEntity(retiringModel);retiringModel=gz::sim::kNullEntity;
    }
    // Physics discovers newly created model links one iteration later.
    // Create the fixed constraint only after that registration has occurred.
    if(pendingChild!=gz::sim::kNullEntity) {
      joint=ecm.CreateEntity();
      ecm.CreateComponent(joint,gz::sim::components::DetachableJoint({bucket.Entity(),pendingChild,"fixed"}));
      pendingChild=gz::sim::kNullEntity;
    }
    if(std::abs(desired-volume)<1e-9) return;
    if(desired>1e-9 && simTime-lastUpdate<0.05) return;
    lastUpdate=simTime;
    if(joint!=gz::sim::kNullEntity) ecm.RequestRemoveEntity(joint);
    if(model!=gz::sim::kNullEntity) retiringModel=model;
    joint=model=gz::sim::kNullEntity;
    volume=desired; properties=Payload(volume,density);
    if(properties.mass<1e-6) return;
    sdf::Link link; link.SetName("payload");
    gz::math::MassMatrix3d mass;
    mass.SetMass(properties.mass);mass.SetDiagonalMoments(properties.diagonal);
    link.SetInertial(gz::math::Inertiald(mass,gz::math::Pose3d::Zero));
    sdf::Model body;body.SetName("loader_payload_"+std::to_string(++sequence));
    body.SetRawPose(pose*gz::math::Pose3d(properties.center,gz::math::Quaterniond::Identity));
    body.AddLink(link);
    model=creator.CreateEntities(&body);creator.SetParent(model,world);
    const auto child=gz::sim::Model(model).LinkByName(ecm,"payload");
    // Newly captured soil co-moves with the bucket after the cutting impulse.
    // Initialise only the new body, never reset a vehicle joint or vehicle pose.
    const auto linear=bucket.WorldLinearVelocity(ecm,properties.center);
    const auto angular=bucket.WorldAngularVelocity(ecm);
    if(linear) gz::sim::Link(child).SetLinearVelocity(ecm,pose.Rot().RotateVectorReverse(*linear));
    if(angular) gz::sim::Link(child).SetAngularVelocity(ecm,pose.Rot().RotateVectorReverse(*angular));
    pendingChild=child;
  }
private:
  gz::sim::Entity joint=gz::sim::kNullEntity,model=gz::sim::kNullEntity;
  gz::sim::Entity pendingChild=gz::sim::kNullEntity;
  gz::sim::Entity retiringModel=gz::sim::kNullEntity;
  unsigned long sequence=0;
  double lastUpdate=-1;
};
}
