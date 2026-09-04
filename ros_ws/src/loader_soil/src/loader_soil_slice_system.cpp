#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include <loader_sim_msgs/msg/bucket_interaction.hpp>
#include <loader_sim_msgs/msg/terrain_state.hpp>
#include <loader_sim_msgs/msg/vehicle_command.hpp>

namespace loader_soil
{
namespace
{
builtin_interfaces::msg::Time SimTimeMessage(const std::chrono::steady_clock::duration &time)
{
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time).count();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}
}  // namespace

class LoaderSoilSliceSystem final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  ~LoaderSoilSliceSystem() override
  {
    if (context_ && context_->is_valid())
      context_->shutdown("loader soil slice plugin unloaded");
  }

  void Configure(
      const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm,
      gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(entity);
    if (!model_.Valid(ecm))
    {
      gzerr << "LoaderSoilSliceSystem must be attached to a model.\n";
      return;
    }
    bucketLink_ = gz::sim::Link(model_.LinkByName(ecm, "bucket"));
    if (!bucketLink_.Valid(ecm))
    {
      gzerr << "LoaderSoilSliceSystem cannot find bucket link.\n";
      return;
    }
    bucketLink_.EnableVelocityChecks(ecm);
    bucketTiltJoint_ = gz::sim::Joint(model_.JointByName(ecm, "bucket_tilt_joint"));
    if (!bucketTiltJoint_.Valid(ecm))
    {
      gzerr << "LoaderSoilSliceSystem cannot find bucket_tilt_joint.\n";
      return;
    }
    bucketTiltJoint_.EnablePositionCheck(ecm);

    ReadParameter(sdf, "domain_min_m", domainMinM_);
    ReadParameter(sdf, "domain_max_m", domainMaxM_);
    ReadParameter(sdf, "cell_size_m", cellSizeM_);
    ReadParameter(sdf, "slice_width_m", sliceWidthM_);
    ReadParameter(sdf, "pile_center_m", pileCenterM_);
    ReadParameter(sdf, "pile_height_m", pileHeightM_);
    ReadParameter(sdf, "angle_of_repose_deg", angleOfReposeDeg_);
    ReadParameter(sdf, "bucket_capacity_m3", bucketCapacityM3_);
    ReadParameter(sdf, "bulk_density_kg_m3", bulkDensityKgM3_);
    ReadParameter(sdf, "visualization_update_hz", visualizationUpdateHz_);
    ReadParameter(sdf, "visualization_column_height_m", visualizationColumnHeightM_);
    ReadParameter(sdf, "unload_tilt_threshold_rad", unloadTiltThresholdRad_);
    ReadParameter(sdf, "maximum_unload_rate_m3ps", maximumUnloadRateM3ps_);

    const std::size_t cellCount = static_cast<std::size_t>(
        std::ceil((domainMaxM_ - domainMinM_) / cellSizeM_));
    heightsM_.resize(cellCount, 0.0);
    const double slope = std::tan(angleOfReposeDeg_ * M_PI / 180.0);
    for (std::size_t index = 0; index < heightsM_.size(); ++index)
    {
      const double x = domainMinM_ + (static_cast<double>(index) + 0.5) * cellSizeM_;
      heightsM_[index] = std::max(0.0, pileHeightM_ - slope * std::abs(x - pileCenterM_));
    }
    initialVolumeM3_ = TerrainVolume();

    visualColumnEntities_.resize(heightsM_.size(), gz::sim::kNullEntity);
    std::size_t visualColumnCount = 0;
    for (std::size_t index = 0; index < heightsM_.size(); ++index)
    {
      std::ostringstream name;
      name << "soil_column_" << std::setfill('0') << std::setw(3) << index;
      const auto columnEntity = ecm.EntityByComponents(
          gz::sim::components::Name(name.str()), gz::sim::components::Model());
      if (columnEntity != gz::sim::kNullEntity)
      {
        visualColumnEntities_[index] = columnEntity;
        ++visualColumnCount;
      }
    }

    try
    {
      context_ = std::make_shared<rclcpp::Context>();
      context_->init(0, nullptr);
      rclcpp::NodeOptions options;
      options.context(context_);
      options.start_parameter_services(false);
      options.start_parameter_event_publisher(false);
      node_ = std::make_shared<rclcpp::Node>(
          "loader_soil_slice_" + std::to_string(entity), options);
      interactionPublisher_ = node_->create_publisher<loader_sim_msgs::msg::BucketInteraction>(
          "/loader/bucket_interaction", rclcpp::QoS(10).reliable());
      terrainPublisher_ = node_->create_publisher<loader_sim_msgs::msg::TerrainState>(
          "/loader/terrain_state", rclcpp::QoS(10).reliable());
      commandSubscription_ = node_->create_subscription<loader_sim_msgs::msg::VehicleCommand>(
          "/loader/command",
          rclcpp::QoS(10).reliable(),
          [this](const loader_sim_msgs::msg::VehicleCommand::SharedPtr command)
          {
            tiltValveCommand_.store(command->tilt_valve_command, std::memory_order_relaxed);
          });
      rclcpp::ExecutorOptions executorOptions;
      executorOptions.context = context_;
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
      executor_->add_node(node_);
    }
    catch (const std::exception &error)
    {
      gzerr << "LoaderSoilSliceSystem ROS initialization failed: " << error.what() << "\n";
      return;
    }
    configured_ = true;
    gzmsg << "LoaderSoilSliceSystem configured: initial_volume=" << initialVolumeM3_
          << " m3, visual_columns=" << visualColumnCount << ", status=nominal\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_ || info.paused)
      return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!(dt > 0.0))
      return;
    if (executor_)
      executor_->spin_some(std::chrono::nanoseconds(0));
    const auto pose = bucketLink_.WorldPose(ecm);
    const auto velocity = bucketLink_.WorldLinearVelocity(ecm, cuttingEdgeLocalM_);
    if (!pose || !velocity)
      return;

    const gz::math::Vector3d edgeWorld =
        pose->Pos() + pose->Rot().RotateVector(cuttingEdgeLocalM_);
    maximumPenetrationM_ = 0.0;
    activeCuttingAreaM2_ = 0.0;
    materialInflowM3ps_ = 0.0;
    materialOutflowM3ps_ = 0.0;
    bucketForceWorldN_.Set(0.0, 0.0, 0.0);
    bucketTorqueWorldNm_.Set(0.0, 0.0, 0.0);

    const auto tiltPosition = bucketTiltJoint_.Position(ecm);
    const double tiltAngle = tiltPosition && !tiltPosition->empty() ? tiltPosition->front() : 0.0;
    const bool unloading =
        tiltValveCommand_.load(std::memory_order_relaxed) <= -0.5 &&
        tiltAngle <= unloadTiltThresholdRad_;

    if (havePreviousEdge_ && !unloading && velocity->X() > 0.02)
    {
      const double deltaX = edgeWorld.X() - previousEdgeWorldM_.X();
      const double planarDistance = std::hypot(deltaX, edgeWorld.Z() - previousEdgeWorldM_.Z());
      const int substeps = std::max(1, static_cast<int>(std::ceil(planarDistance / 0.01)));
      double removedVolume = 0.0;
      double peakPenetration = 0.0;
      for (int step = 0; step < substeps; ++step)
      {
        const double ratio0 = static_cast<double>(step) / substeps;
        const double ratio1 = static_cast<double>(step + 1) / substeps;
        const double x0 = previousEdgeWorldM_.X() + deltaX * ratio0;
        const double x1 = previousEdgeWorldM_.X() + deltaX * ratio1;
        const double z = previousEdgeWorldM_.Z() +
            (edgeWorld.Z() - previousEdgeWorldM_.Z()) * 0.5 * (ratio0 + ratio1);
        if (x1 <= x0 || std::abs(edgeWorld.Y()) > 0.5 * sliceWidthM_)
          continue;
        const auto cell = CellIndex(0.5 * (x0 + x1));
        if (!cell)
          continue;
        const double penetration = std::max(0.0, heightsM_[*cell] - z);
        peakPenetration = std::max(peakPenetration, penetration);
        const double availableCapacity = bucketCapacityM3_ - payloadVolumeM3_;
        const double proposedVolume = penetration * (x1 - x0) * sliceWidthM_;
        const double cellVolume = heightsM_[*cell] * cellSizeM_ * sliceWidthM_;
        const double actualVolume = std::min(
            std::max(0.0, proposedVolume),
            std::max(0.0, std::min(availableCapacity, cellVolume)));
        if (actualVolume > 0.0)
        {
          heightsM_[*cell] -= actualVolume / (cellSizeM_ * sliceWidthM_);
          payloadVolumeM3_ += actualVolume;
          excavatedVolumeM3_ += actualVolume;
          removedVolume += actualVolume;
        }
      }
      maximumPenetrationM_ = peakPenetration;
      activeCuttingAreaM2_ = peakPenetration * sliceWidthM_;
      materialInflowM3ps_ = removedVolume / dt;

      if (peakPenetration > 0.0 && velocity->X() > 0.02)
      {
        const double passiveCoefficient = (1.0 + std::sin(internalFrictionAngleRad_)) /
            (1.0 - std::sin(internalFrictionAngleRad_));
        const double unitWeight = bulkDensityKgM3_ * gravityMps2_;
        const double speedRatio = velocity->X() / referenceSpeedMps_;
        const double forceMagnitude = std::min(
            maximumCuttingForceN_,
            0.5 * unitWeight * peakPenetration * peakPenetration * passiveCoefficient *
                sliceWidthM_ * (1.0 + dynamicCoefficient_ * speedRatio * speedRatio));
        const double upliftAngle = rakeAngleRad_ - soilToolFrictionAngleRad_;
        bucketForceWorldN_.Set(
            -forceMagnitude * std::cos(upliftAngle),
            0.0,
            forceMagnitude * std::sin(upliftAngle));
        const gz::math::Vector3d offsetWorld = pose->Rot().RotateVector(cuttingEdgeLocalM_);
        bucketTorqueWorldNm_ = offsetWorld.Cross(bucketForceWorldN_);
        bucketLink_.AddWorldForce(ecm, bucketForceWorldN_, cuttingEdgeLocalM_);
      }
    }
    previousEdgeWorldM_ = edgeWorld;
    havePreviousEdge_ = true;

    if (unloading && payloadVolumeM3_ > 0.0)
    {
      const double requestedVolume = std::min(payloadVolumeM3_, maximumUnloadRateM3ps_ * dt);
      const double unloadedVolume = DepositAtRepose(edgeWorld.X(), requestedVolume);
      payloadVolumeM3_ -= unloadedVolume;
      dumpedVolumeM3_ += unloadedVolume;
      materialOutflowM3ps_ = unloadedVolume / dt;
    }

    if (payloadVolumeM3_ > 0.0)
    {
      const double payloadMass = payloadVolumeM3_ * bulkDensityKgM3_;
      bucketLink_.AddWorldForce(
          ecm,
          gz::math::Vector3d(0.0, 0.0, -payloadMass * gravityMps2_),
          payloadCenterLocalM_);
    }

    if (visualizationUpdateHz_ > 0.0 &&
        (lastVisualizationTime_.count() == 0 ||
         std::chrono::duration<double>(info.simTime - lastVisualizationTime_).count() >=
             1.0 / visualizationUpdateHz_))
    {
      lastVisualizationTime_ = info.simTime;
      for (std::size_t index = 0; index < visualColumnEntities_.size(); ++index)
      {
        if (visualColumnEntities_[index] == gz::sim::kNullEntity)
          continue;
        const double x = domainMinM_ + (static_cast<double>(index) + 0.5) * cellSizeM_;
        const double z = heightsM_[index] - 0.5 * visualizationColumnHeightM_;
        gz::sim::Model(visualColumnEntities_[index]).SetWorldPoseCmd(
            ecm, gz::math::Pose3d(x, 0.0, z, 0.0, 0.0, 0.0));
      }
    }
  }

  void PostUpdate(
      const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &) override
  {
    if (!configured_ || info.paused || !interactionPublisher_ || !terrainPublisher_)
      return;
    if (lastPublishTime_.count() != 0 &&
        std::chrono::duration<double>(info.simTime - lastPublishTime_).count() < 0.02)
    {
      return;
    }
    lastPublishTime_ = info.simTime;
    const auto stamp = SimTimeMessage(info.simTime);

    loader_sim_msgs::msg::BucketInteraction interaction;
    interaction.header.stamp = stamp;
    interaction.header.frame_id = "world";
    interaction.bucket_wrench.force.x = bucketForceWorldN_.X();
    interaction.bucket_wrench.force.y = bucketForceWorldN_.Y();
    interaction.bucket_wrench.force.z = bucketForceWorldN_.Z();
    interaction.bucket_wrench.torque.x = bucketTorqueWorldNm_.X();
    interaction.bucket_wrench.torque.y = bucketTorqueWorldNm_.Y();
    interaction.bucket_wrench.torque.z = bucketTorqueWorldNm_.Z();
    interaction.maximum_penetration_depth_m = maximumPenetrationM_;
    interaction.active_cutting_area_m2 = activeCuttingAreaM2_;
    interaction.material_inflow_m3ps = materialInflowM3ps_;
    interaction.material_outflow_m3ps = materialOutflowM3ps_;
    interaction.bucket_material_volume_m3 = payloadVolumeM3_;
    interaction.bucket_material_mass_kg = payloadVolumeM3_ * bulkDensityKgM3_;
    interactionPublisher_->publish(interaction);

    loader_sim_msgs::msg::TerrainState terrain;
    terrain.header.stamp = stamp;
    terrain.header.frame_id = "world";
    terrain.material_type = "dry_sand_nominal";
    terrain.domain_min_m = domainMinM_;
    terrain.cell_size_m = cellSizeM_;
    terrain.slice_width_m = sliceWidthM_;
    terrain.height_profile_m = heightsM_;
    terrain.initial_volume_m3 = initialVolumeM3_;
    terrain.remaining_volume_m3 = TerrainVolume();
    terrain.excavated_volume_m3 = excavatedVolumeM3_;
    terrain.dumped_volume_m3 = dumpedVolumeM3_;
    const double balance = terrain.remaining_volume_m3 + payloadVolumeM3_ - initialVolumeM3_;
    terrain.relative_volume_conservation_error =
        initialVolumeM3_ > 0.0 ? balance / initialVolumeM3_ : 0.0;
    terrain.relative_mass_conservation_error = terrain.relative_volume_conservation_error;
    terrain.update_sequence = ++updateSequence_;
    terrainPublisher_->publish(terrain);
  }

private:
  template<typename T>
  static void ReadParameter(
      const std::shared_ptr<const sdf::Element> &sdf,
      const std::string &name,
      T &value)
  {
    if (sdf->HasElement(name))
      value = sdf->Get<T>(name);
  }

  std::optional<std::size_t> CellIndex(double x) const
  {
    const auto index = static_cast<long>(std::floor((x - domainMinM_) / cellSizeM_));
    if (index < 0 || index >= static_cast<long>(heightsM_.size()))
      return std::nullopt;
    return static_cast<std::size_t>(index);
  }

  double TerrainVolume() const
  {
    double heightSum = 0.0;
    for (const double height : heightsM_)
      heightSum += height;
    return heightSum * cellSizeM_ * sliceWidthM_;
  }

  double DepositAtRepose(double centerM, double requestedVolumeM3)
  {
    if (!(requestedVolumeM3 > 0.0))
      return 0.0;
    const std::vector<double> original = heightsM_;
    const double targetAreaM2 = requestedVolumeM3 / sliceWidthM_;
    const double slope = std::tan(angleOfReposeDeg_ * M_PI / 180.0);
    auto addedArea = [&](double apexHeightM)
    {
      double area = 0.0;
      for (std::size_t index = 0; index < original.size(); ++index)
      {
        const double x = domainMinM_ + (static_cast<double>(index) + 0.5) * cellSizeM_;
        area += std::max(
            0.0, apexHeightM - slope * std::abs(x - centerM) - original[index]);
      }
      return area * cellSizeM_;
    };

    double lower = 0.0;
    double upper = *std::max_element(original.begin(), original.end()) + 1.0;
    while (addedArea(upper) < targetAreaM2)
      upper *= 2.0;
    for (int iteration = 0; iteration < 60; ++iteration)
    {
      const double middle = 0.5 * (lower + upper);
      if (addedArea(middle) < targetAreaM2)
        lower = middle;
      else
        upper = middle;
    }
    const double apex = 0.5 * (lower + upper);
    for (std::size_t index = 0; index < heightsM_.size(); ++index)
    {
      const double x = domainMinM_ + (static_cast<double>(index) + 0.5) * cellSizeM_;
      heightsM_[index] = std::max(
          original[index], apex - slope * std::abs(x - centerM));
    }
    return requestedVolumeM3;
  }

  gz::sim::Model model_;
  gz::sim::Link bucketLink_;
  gz::sim::Joint bucketTiltJoint_;
  bool configured_{false};
  bool havePreviousEdge_{false};
  gz::math::Vector3d previousEdgeWorldM_;
  const gz::math::Vector3d cuttingEdgeLocalM_{1.025, 0.0, -0.625};
  const gz::math::Vector3d payloadCenterLocalM_{0.35, 0.0, -0.05};

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Publisher<loader_sim_msgs::msg::BucketInteraction>::SharedPtr interactionPublisher_;
  rclcpp::Publisher<loader_sim_msgs::msg::TerrainState>::SharedPtr terrainPublisher_;
  rclcpp::Subscription<loader_sim_msgs::msg::VehicleCommand>::SharedPtr commandSubscription_;
  std::atomic<double> tiltValveCommand_{0.0};

  std::vector<double> heightsM_;
  std::vector<gz::sim::Entity> visualColumnEntities_;
  double domainMinM_{0.0};
  double domainMaxM_{14.0};
  double cellSizeM_{0.05};
  double sliceWidthM_{2.7};
  double pileCenterM_{6.0};
  double pileHeightM_{1.8};
  double angleOfReposeDeg_{34.0};
  double bucketCapacityM3_{3.0};
  double bulkDensityKgM3_{1600.0};
  double visualizationUpdateHz_{10.0};
  double visualizationColumnHeightM_{1.8};
  double unloadTiltThresholdRad_{-0.35};
  double maximumUnloadRateM3ps_{3.0};
  double gravityMps2_{9.80665};
  double internalFrictionAngleRad_{34.0 * M_PI / 180.0};
  double soilToolFrictionAngleRad_{20.0 * M_PI / 180.0};
  double rakeAngleRad_{35.0 * M_PI / 180.0};
  double dynamicCoefficient_{0.15};
  double referenceSpeedMps_{1.0};
  double maximumCuttingForceN_{300000.0};

  double initialVolumeM3_{0.0};
  double payloadVolumeM3_{0.0};
  double excavatedVolumeM3_{0.0};
  double dumpedVolumeM3_{0.0};
  double maximumPenetrationM_{0.0};
  double activeCuttingAreaM2_{0.0};
  double materialInflowM3ps_{0.0};
  double materialOutflowM3ps_{0.0};
  gz::math::Vector3d bucketForceWorldN_;
  gz::math::Vector3d bucketTorqueWorldNm_;
  std::chrono::steady_clock::duration lastPublishTime_{};
  std::chrono::steady_clock::duration lastVisualizationTime_{};
  std::uint64_t updateSequence_{0};
};
}  // namespace loader_soil

GZ_ADD_PLUGIN(
    loader_soil::LoaderSoilSliceSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(loader_soil::LoaderSoilSliceSystem, "loader_soil::LoaderSoilSliceSystem")
