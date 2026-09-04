#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <loader_sim_msgs/msg/vehicle_command.hpp>
#include <loader_sim_msgs/msg/vehicle_state.hpp>

namespace loader_sim
{
namespace
{
constexpr std::uint32_t kFaultCommandTimeout = 1U << 0;
constexpr std::uint32_t kFaultInvalidNumeric = 1U << 1;
constexpr std::uint32_t kFaultCommandSaturated = 1U << 2;
constexpr std::uint32_t kFaultInvalidGear = 1U << 3;
constexpr std::uint32_t kFaultMissingJoint = 1U << 4;

constexpr std::size_t kFrontLeft = 0;
constexpr std::size_t kFrontRight = 1;
constexpr std::size_t kRearLeft = 2;
constexpr std::size_t kRearRight = 3;
constexpr std::size_t kArticulation = 4;
constexpr std::size_t kRearAxle = 5;
constexpr std::size_t kLift = 6;
constexpr std::size_t kTilt = 7;

constexpr std::array<const char *, 8> kJointNames{
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
    "articulation_joint",
    "rear_axle_oscillation_joint",
    "lift_joint",
    "bucket_tilt_joint",
};

double Clamp(double value, double lower, double upper, bool &saturated)
{
  const double result = std::clamp(value, lower, upper);
  saturated = saturated || result != value;
  return result;
}

std::pair<double, double> Rotate(double x, double z, double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return {cosine * x - sine * z, sine * x + cosine * z};
}

std::pair<double, double> RotateDerivative(double x, double z, double angle)
{
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return {-sine * x - cosine * z, cosine * x - sine * z};
}

std::pair<double, double> LengthAndRate(
    double x, double z, double dx, double dz)
{
  const double length = std::hypot(x, z);
  if (length <= 1.0e-9)
    return {length, 0.0};
  return {length, (x * dx + z * dz) / length};
}

std::pair<double, double> LiftCylinderKinematics(double liftAngle)
{
  constexpr double pivotX = 0.75;
  constexpr double pivotZ = 0.65;
  constexpr double baseX = 1.20;
  constexpr double baseZ = 0.0;
  const auto rod = Rotate(1.10, -0.15, liftAngle);
  const auto rodRate = RotateDerivative(1.10, -0.15, liftAngle);
  return LengthAndRate(
      pivotX + rod.first - baseX,
      pivotZ + rod.second - baseZ,
      rodRate.first,
      rodRate.second);
}

std::pair<double, double> TiltCylinderKinematics(double tiltAngle)
{
  constexpr double bucketPivotX = 2.80;
  constexpr double bucketPivotZ = 0.0;
  constexpr double baseX = 2.0;
  constexpr double baseZ = 0.25;
  const auto rod = Rotate(0.25, 0.50, tiltAngle);
  const auto rodRate = RotateDerivative(0.25, 0.50, tiltAngle);
  return LengthAndRate(
      bucketPivotX + rod.first - baseX,
      bucketPivotZ + rod.second - baseZ,
      rodRate.first,
      rodRate.second);
}

double FirstOrZero(const std::optional<std::vector<double>> &values)
{
  return values && !values->empty() ? values->front() : 0.0;
}

builtin_interfaces::msg::Time SimTimeMessage(const std::chrono::steady_clock::duration &time)
{
  const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time).count();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}
}  // namespace

class LoaderDynamicsSystem final :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
  ~LoaderDynamicsSystem() override
  {
    if (executor_ && node_)
      executor_->remove_node(node_);
    if (context_ && context_->is_valid())
      context_->shutdown("loader dynamics plugin unloaded");
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
      gzerr << "LoaderDynamicsSystem must be attached to a model.\n";
      faultFlags_ |= kFaultMissingJoint;
      return;
    }

    if (sdf->HasElement("command_topic"))
      commandTopic_ = sdf->Get<std::string>("command_topic");
    if (sdf->HasElement("state_topic"))
      stateTopic_ = sdf->Get<std::string>("state_topic");
    if (sdf->HasElement("command_timeout_s"))
      commandTimeoutS_ = sdf->Get<double>("command_timeout_s");
    if (sdf->HasElement("state_rate_hz"))
      stateRateHz_ = sdf->Get<double>("state_rate_hz");

    bool jointsReady = true;
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
      const gz::sim::Entity jointEntity = model_.JointByName(ecm, kJointNames[index]);
      joints_[index] = gz::sim::Joint(jointEntity);
      if (!joints_[index].Valid(ecm))
      {
        gzerr << "LoaderDynamicsSystem missing joint: " << kJointNames[index] << "\n";
        jointsReady = false;
        continue;
      }
      joints_[index].EnablePositionCheck(ecm);
      joints_[index].EnableVelocityCheck(ecm);
    }
    configured_ = jointsReady;
    if (!configured_)
      faultFlags_ |= kFaultMissingJoint;

    try
    {
      context_ = std::make_shared<rclcpp::Context>();
      context_->init(0, nullptr);
      rclcpp::NodeOptions nodeOptions;
      nodeOptions.context(context_);
      nodeOptions.start_parameter_services(false);
      nodeOptions.start_parameter_event_publisher(false);
      node_ = std::make_shared<rclcpp::Node>(
          "loader_dynamics_" + std::to_string(entity), nodeOptions);

      commandSubscription_ = node_->create_subscription<loader_sim_msgs::msg::VehicleCommand>(
          commandTopic_, rclcpp::QoS(10).reliable(),
          [this](const loader_sim_msgs::msg::VehicleCommand::SharedPtr message)
          {
            if (!CommandFinite(*message))
            {
              faultFlags_ |= kFaultInvalidNumeric;
              return;
            }
            if (message->gear < loader_sim_msgs::msg::VehicleCommand::GEAR_REVERSE ||
                message->gear > loader_sim_msgs::msg::VehicleCommand::GEAR_FORWARD)
            {
              faultFlags_ |= kFaultInvalidGear;
              return;
            }
            command_ = *message;
            commandUpdated_ = true;
          });
      statePublisher_ = node_->create_publisher<loader_sim_msgs::msg::VehicleState>(
          stateTopic_, rclcpp::QoS(10).reliable());

      rclcpp::ExecutorOptions executorOptions;
      executorOptions.context = context_;
      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
      executor_->add_node(node_);
    }
    catch (const std::exception &error)
    {
      gzerr << "Failed to initialize ROS for LoaderDynamicsSystem: " << error.what() << "\n";
      configured_ = false;
    }

    gzmsg << "LoaderDynamicsSystem configured with nominal parameters. command="
          << commandTopic_ << " state=" << stateTopic_ << "\n";
  }

  void PreUpdate(
      const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_ || !executor_)
      return;

    executor_->spin_some(std::chrono::nanoseconds(0));
    if (commandUpdated_)
    {
      lastCommandSimTime_ = info.simTime;
      receivedCommand_ = true;
      commandUpdated_ = false;
    }

    if (info.paused)
      return;

    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!(dt > 0.0) || !std::isfinite(dt))
      return;

    const bool timedOut = !receivedCommand_ ||
        std::chrono::duration<double>(info.simTime - lastCommandSimTime_).count() > commandTimeoutS_;
    activeFaultFlags_ = faultFlags_;
    if (timedOut)
      activeFaultFlags_ |= kFaultCommandTimeout;

    bool saturated = false;
    const bool emergencyStop = timedOut || command_.emergency_stop;
    const double brakeCommand = emergencyStop ? 1.0 : Clamp(command_.brake_command, 0.0, 1.0, saturated);
    double tractionTorque = Clamp(command_.traction_torque_nm, -maximumTractionTorqueNm_, maximumTractionTorqueNm_, saturated);
    if (emergencyStop || command_.gear == loader_sim_msgs::msg::VehicleCommand::GEAR_NEUTRAL)
      tractionTorque = 0.0;

    for (std::size_t index = kFrontLeft; index <= kRearRight; ++index)
    {
      const double velocity = FirstOrZero(joints_[index].Velocity(ecm));
      const double brakeTorque = -std::clamp(velocity / 0.1, -1.0, 1.0) *
          brakeCommand * maximumBrakeTorqueNm_ / 4.0;
      efforts_[index] = tractionTorque / 4.0 + brakeTorque;
      joints_[index].SetForce(ecm, {efforts_[index]});
    }

    const double articulationPosition = FirstOrZero(joints_[kArticulation].Position(ecm));
    const double articulationVelocity = FirstOrZero(joints_[kArticulation].Velocity(ecm));
    const double articulationTarget = Clamp(
        command_.target_articulation_angle_rad, -0.698132, 0.698132, saturated);
    efforts_[kArticulation] = std::clamp(
        articulationKp_ * (articulationTarget - articulationPosition) -
            articulationKd_ * articulationVelocity,
        -maximumArticulationTorqueNm_, maximumArticulationTorqueNm_);
    if (emergencyStop)
      efforts_[kArticulation] = -articulationKd_ * articulationVelocity;
    joints_[kArticulation].SetForce(ecm, {efforts_[kArticulation]});

    const double liftPosition = FirstOrZero(joints_[kLift].Position(ecm));
    const double liftVelocity = FirstOrZero(joints_[kLift].Velocity(ecm));
    const double tiltPosition = FirstOrZero(joints_[kTilt].Position(ecm));
    const double tiltVelocity = FirstOrZero(joints_[kTilt].Velocity(ecm));

    const double liftValve = emergencyStop ? 0.0 : Clamp(command_.lift_valve_command, -1.0, 1.0, saturated);
    const double tiltValve = emergencyStop ? 0.0 : Clamp(command_.tilt_valve_command, -1.0, 1.0, saturated);
    const double pressureAlpha = 1.0 - std::exp(-dt / hydraulicPressureTimeConstantS_);
    liftPressurePa_ += pressureAlpha * (liftValve * reliefPressurePa_ - liftPressurePa_);
    tiltPressurePa_ += pressureAlpha * (-tiltValve * reliefPressurePa_ - tiltPressurePa_);
    liftPressurePa_ = std::clamp(liftPressurePa_, -reliefPressurePa_, reliefPressurePa_);
    tiltPressurePa_ = std::clamp(tiltPressurePa_, -reliefPressurePa_, reliefPressurePa_);

    const auto liftCylinder = LiftCylinderKinematics(liftPosition);
    const auto tiltCylinder = TiltCylinderKinematics(tiltPosition);
    efforts_[kLift] = std::clamp(
        liftPressurePa_ * liftPistonAreaM2_ * liftCylinderCount_ * liftCylinder.second -
            liftDampingNms_ * liftVelocity,
        -maximumLiftTorqueNm_, maximumLiftTorqueNm_);
    efforts_[kTilt] = std::clamp(
        tiltPressurePa_ * tiltPistonAreaM2_ * tiltCylinderCount_ * tiltCylinder.second -
            tiltDampingNms_ * tiltVelocity,
        -maximumTiltTorqueNm_, maximumTiltTorqueNm_);
    joints_[kLift].SetForce(ecm, {efforts_[kLift]});
    joints_[kTilt].SetForce(ecm, {efforts_[kTilt]});

    efforts_[kRearAxle] = 0.0;
    if (saturated)
      activeFaultFlags_ |= kFaultCommandSaturated;
    emergencyStopActive_ = emergencyStop;
  }

  void PostUpdate(
      const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override
  {
    if (!configured_ || !statePublisher_ || info.paused)
      return;

    const double publishPeriodS = 1.0 / std::max(stateRateHz_, 1.0);
    if (lastStatePublishTime_.count() != 0 &&
        std::chrono::duration<double>(info.simTime - lastStatePublishTime_).count() < publishPeriodS)
      return;
    lastStatePublishTime_ = info.simTime;

    loader_sim_msgs::msg::VehicleState state;
    state.header.stamp = SimTimeMessage(info.simTime);
    state.header.frame_id = "base_link";
    state.joint_state.header = state.header;

    std::array<double, 8> positions{};
    std::array<double, 8> velocities{};
    for (std::size_t index = 0; index < joints_.size(); ++index)
    {
      positions[index] = FirstOrZero(joints_[index].Position(ecm));
      velocities[index] = FirstOrZero(joints_[index].Velocity(ecm));
      state.joint_state.name.emplace_back(kJointNames[index]);
      state.joint_state.position.push_back(positions[index]);
      state.joint_state.velocity.push_back(velocities[index]);
      state.joint_state.effort.push_back(efforts_[index]);
    }

    state.wheel_speed_radps = {
        velocities[kFrontLeft], velocities[kFrontRight],
        velocities[kRearLeft], velocities[kRearRight]};
    state.longitudinal_speed_mps = 0.75 * 0.25 *
        (velocities[kFrontLeft] + velocities[kFrontRight] +
         velocities[kRearLeft] + velocities[kRearRight]);

    constexpr double liftRetractedLengthM = 0.716786;
    constexpr double tiltRetractedLengthM = 0.674473;
    state.lift_cylinder_position_m = LiftCylinderKinematics(positions[kLift]).first - liftRetractedLengthM;
    state.tilt_cylinder_position_m = TiltCylinderKinematics(positions[kTilt]).first - tiltRetractedLengthM;
    state.lift_cylinder_pressure_pa = liftPressurePa_;
    state.tilt_cylinder_pressure_pa = tiltPressurePa_;
    state.bucket_payload_mass_kg = 0.0;
    state.bucket_payload_center_of_mass_m.x = 0.0;
    state.bucket_payload_center_of_mass_m.y = 0.0;
    state.bucket_payload_center_of_mass_m.z = 0.0;
    state.fault_flags = activeFaultFlags_;
    state.emergency_stop_active = emergencyStopActive_;
    statePublisher_->publish(state);
  }

private:
  static bool CommandFinite(const loader_sim_msgs::msg::VehicleCommand &command)
  {
    return std::isfinite(command.traction_torque_nm) &&
        std::isfinite(command.brake_command) &&
        std::isfinite(command.target_articulation_angle_rad) &&
        std::isfinite(command.lift_valve_command) &&
        std::isfinite(command.tilt_valve_command);
  }

  gz::sim::Model model_;
  std::array<gz::sim::Joint, 8> joints_;
  std::array<double, 8> efforts_{};

  std::shared_ptr<rclcpp::Context> context_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<loader_sim_msgs::msg::VehicleCommand>::SharedPtr commandSubscription_;
  rclcpp::Publisher<loader_sim_msgs::msg::VehicleState>::SharedPtr statePublisher_;

  loader_sim_msgs::msg::VehicleCommand command_;
  bool configured_{false};
  bool receivedCommand_{false};
  bool commandUpdated_{false};
  bool emergencyStopActive_{true};
  std::uint32_t faultFlags_{0};
  std::uint32_t activeFaultFlags_{kFaultCommandTimeout};
  std::chrono::steady_clock::duration lastCommandSimTime_{};
  std::chrono::steady_clock::duration lastStatePublishTime_{};

  std::string commandTopic_{"/loader/command"};
  std::string stateTopic_{"/loader/state"};
  double commandTimeoutS_{0.5};
  double stateRateHz_{50.0};

  double liftPressurePa_{0.0};
  double tiltPressurePa_{0.0};
  double maximumTractionTorqueNm_{80000.0};
  double maximumBrakeTorqueNm_{120000.0};
  double maximumArticulationTorqueNm_{250000.0};
  double maximumLiftTorqueNm_{600000.0};
  double maximumTiltTorqueNm_{500000.0};
  double articulationKp_{180000.0};
  double articulationKd_{30000.0};
  double liftDampingNms_{30000.0};
  double tiltDampingNms_{25000.0};
  double reliefPressurePa_{25000000.0};
  double hydraulicPressureTimeConstantS_{0.08};
  double liftPistonAreaM2_{0.035};
  double tiltPistonAreaM2_{0.030};
  double liftCylinderCount_{2.0};
  double tiltCylinderCount_{1.0};
};
}  // namespace loader_sim

GZ_ADD_PLUGIN(
    loader_sim::LoaderDynamicsSystem,
    gz::sim::System,
    gz::sim::ISystemConfigure,
    gz::sim::ISystemPreUpdate,
    gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(loader_sim::LoaderDynamicsSystem, "loader_sim::LoaderDynamicsSystem")
