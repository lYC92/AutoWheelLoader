#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include <loader_sim_msgs/msg/vehicle_command.hpp>
#include <loader_sim_msgs/msg/vehicle_state.hpp>

namespace loader_control
{
namespace
{
constexpr std::uint32_t kFaultCommandTimeout = 1U << 0;
constexpr std::uint32_t kFaultInvalidNumeric = 1U << 1;
constexpr std::uint32_t kFaultCommandSaturated = 1U << 2;
constexpr std::uint32_t kFaultInvalidGear = 1U << 3;

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

constexpr std::array<std::size_t, 7> kCommandJointIndices{
    kFrontLeft, kFrontRight, kRearLeft, kRearRight, kArticulation, kLift, kTilt};

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

std::pair<double, double> LengthAndRate(double x, double z, double dx, double dz)
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

bool CommandFinite(const loader_sim_msgs::msg::VehicleCommand &command)
{
  return std::isfinite(command.traction_torque_nm) &&
      std::isfinite(command.brake_command) &&
      std::isfinite(command.target_articulation_angle_rad) &&
      std::isfinite(command.lift_valve_command) &&
      std::isfinite(command.tilt_valve_command);
}

struct BufferedCommand
{
  loader_sim_msgs::msg::VehicleCommand message;
  std::uint64_t sequence{0};
  bool valid{false};
};
}  // namespace

class LoaderCommandController final : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const std::size_t index : kCommandJointIndices)
    {
      configuration.names.emplace_back(
          std::string(kJointNames[index]) + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return configuration;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration configuration;
    configuration.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const char *jointName : kJointNames)
    {
      configuration.names.emplace_back(
          std::string(jointName) + "/" + hardware_interface::HW_IF_POSITION);
      configuration.names.emplace_back(
          std::string(jointName) + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return configuration;
  }

  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &) override
  {
    auto node = get_node();
    commandSubscription_ = node->create_subscription<loader_sim_msgs::msg::VehicleCommand>(
        "/loader/command", rclcpp::QoS(10).reliable(),
        [this](const loader_sim_msgs::msg::VehicleCommand::SharedPtr message)
        {
          if (!CommandFinite(*message))
          {
            latchedFaultFlags_.fetch_or(kFaultInvalidNumeric, std::memory_order_relaxed);
            return;
          }
          if (message->gear < loader_sim_msgs::msg::VehicleCommand::GEAR_REVERSE ||
              message->gear > loader_sim_msgs::msg::VehicleCommand::GEAR_FORWARD)
          {
            latchedFaultFlags_.fetch_or(kFaultInvalidGear, std::memory_order_relaxed);
            return;
          }
          BufferedCommand buffered;
          buffered.message = *message;
          buffered.sequence = nextCommandSequence_.fetch_add(1, std::memory_order_relaxed) + 1;
          buffered.valid = true;
          commandBuffer_.writeFromNonRT(buffered);
        });

    statePublisher_ = node->create_publisher<loader_sim_msgs::msg::VehicleState>(
        "/loader/state", rclcpp::QoS(10).reliable());
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &) override
  {
    statePublisher_->on_activate();
    liftPressurePa_ = 0.0;
    tiltPressurePa_ = 0.0;
    activeCommand_ = loader_sim_msgs::msg::VehicleCommand();
    receivedCommand_ = false;
    consumedCommandSequence_ = 0;
    lastCommandTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    lastStatePublishTime_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    efforts_.fill(0.0);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &) override
  {
    for (auto &commandInterface : command_interfaces_)
      (void)commandInterface.set_value(0.0);
    statePublisher_->on_deactivate();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(
      const rclcpp::Time &time,
      const rclcpp::Duration &period) override
  {
    const BufferedCommand *buffered = commandBuffer_.readFromRT();
    if (buffered && buffered->valid && buffered->sequence != consumedCommandSequence_)
    {
      activeCommand_ = buffered->message;
      consumedCommandSequence_ = buffered->sequence;
      lastCommandTime_ = time;
      receivedCommand_ = true;
    }

    const double dt = period.seconds();
    if (!(dt > 0.0) || !std::isfinite(dt))
      return controller_interface::return_type::OK;

    std::array<double, 8> positions{};
    std::array<double, 8> velocities{};
    for (std::size_t index = 0; index < kJointNames.size(); ++index)
    {
      const auto position = state_interfaces_[2 * index].get_optional<double>();
      const auto velocity = state_interfaces_[2 * index + 1].get_optional<double>();
      positions[index] = position.value_or(0.0);
      velocities[index] = velocity.value_or(0.0);
    }

    const bool timedOut = !receivedCommand_ || (time - lastCommandTime_).seconds() > commandTimeoutS_;
    std::uint32_t activeFaultFlags = latchedFaultFlags_.load(std::memory_order_relaxed);
    if (timedOut)
      activeFaultFlags |= kFaultCommandTimeout;

    bool saturated = false;
    const bool emergencyStop = timedOut || activeCommand_.emergency_stop;
    const double brakeCommand = emergencyStop ? 1.0 :
        Clamp(activeCommand_.brake_command, 0.0, 1.0, saturated);
    double tractionTorque = Clamp(
        activeCommand_.traction_torque_nm,
        -maximumTractionTorqueNm_, maximumTractionTorqueNm_, saturated);
    if (emergencyStop ||
        activeCommand_.gear == loader_sim_msgs::msg::VehicleCommand::GEAR_NEUTRAL)
    {
      tractionTorque = 0.0;
    }

    for (std::size_t index = kFrontLeft; index <= kRearRight; ++index)
    {
      const double brakeTorque = -std::clamp(velocities[index] / 0.1, -1.0, 1.0) *
          brakeCommand * maximumBrakeTorqueNm_ / 4.0;
      efforts_[index] = tractionTorque / 4.0 + brakeTorque;
    }

    const double articulationTarget = Clamp(
        activeCommand_.target_articulation_angle_rad, -0.698132, 0.698132, saturated);
    efforts_[kArticulation] = std::clamp(
        articulationKp_ * (articulationTarget - positions[kArticulation]) -
            articulationKd_ * velocities[kArticulation],
        -maximumArticulationTorqueNm_, maximumArticulationTorqueNm_);
    if (emergencyStop)
    {
      efforts_[kArticulation] = std::clamp(
          -articulationKd_ * velocities[kArticulation],
          -maximumArticulationTorqueNm_, maximumArticulationTorqueNm_);
    }

    const double liftValve = emergencyStop ? 0.0 :
        Clamp(activeCommand_.lift_valve_command, -1.0, 1.0, saturated);
    const double tiltValve = emergencyStop ? 0.0 :
        Clamp(activeCommand_.tilt_valve_command, -1.0, 1.0, saturated);
    const double pressureAlpha = 1.0 - std::exp(-dt / hydraulicPressureTimeConstantS_);
    liftPressurePa_ += pressureAlpha * (liftValve * reliefPressurePa_ - liftPressurePa_);
    tiltPressurePa_ += pressureAlpha * (-tiltValve * reliefPressurePa_ - tiltPressurePa_);
    liftPressurePa_ = std::clamp(liftPressurePa_, -reliefPressurePa_, reliefPressurePa_);
    tiltPressurePa_ = std::clamp(tiltPressurePa_, -reliefPressurePa_, reliefPressurePa_);

    const auto liftCylinder = LiftCylinderKinematics(positions[kLift]);
    const auto tiltCylinder = TiltCylinderKinematics(positions[kTilt]);
    efforts_[kLift] = std::clamp(
        liftPressurePa_ * liftPistonAreaM2_ * liftCylinderCount_ * liftCylinder.second -
            liftDampingNms_ * velocities[kLift],
        -maximumLiftTorqueNm_, maximumLiftTorqueNm_);
    efforts_[kTilt] = std::clamp(
        tiltPressurePa_ * tiltPistonAreaM2_ * tiltCylinderCount_ * tiltCylinder.second -
            tiltDampingNms_ * velocities[kTilt],
        -maximumTiltTorqueNm_, maximumTiltTorqueNm_);
    efforts_[kRearAxle] = 0.0;

    bool writeSucceeded = true;
    for (std::size_t commandIndex = 0; commandIndex < kCommandJointIndices.size(); ++commandIndex)
    {
      writeSucceeded = command_interfaces_[commandIndex].set_value(
          efforts_[kCommandJointIndices[commandIndex]]) && writeSucceeded;
    }
    if (!writeSucceeded)
      return controller_interface::return_type::ERROR;

    if (saturated)
      activeFaultFlags |= kFaultCommandSaturated;

    const bool shouldPublish = lastStatePublishTime_.nanoseconds() == 0 ||
        (time - lastStatePublishTime_).seconds() >= 1.0 / stateRateHz_;
    if (shouldPublish && statePublisher_->is_activated())
    {
      lastStatePublishTime_ = time;
      loader_sim_msgs::msg::VehicleState state;
      state.header.stamp = time;
      state.header.frame_id = "base_link";
      state.joint_state.header = state.header;
      for (std::size_t index = 0; index < kJointNames.size(); ++index)
      {
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
      state.lift_cylinder_position_m =
          LiftCylinderKinematics(positions[kLift]).first - liftRetractedLengthM;
      state.tilt_cylinder_position_m =
          TiltCylinderKinematics(positions[kTilt]).first - tiltRetractedLengthM;
      state.lift_cylinder_pressure_pa = liftPressurePa_;
      state.tilt_cylinder_pressure_pa = tiltPressurePa_;
      state.bucket_payload_mass_kg = 0.0;
      state.fault_flags = activeFaultFlags;
      state.emergency_stop_active = emergencyStop;
      statePublisher_->publish(state);
    }

    return controller_interface::return_type::OK;
  }

private:
  realtime_tools::RealtimeBuffer<BufferedCommand> commandBuffer_;
  std::atomic<std::uint64_t> nextCommandSequence_{0};
  std::atomic<std::uint32_t> latchedFaultFlags_{0};
  loader_sim_msgs::msg::VehicleCommand activeCommand_;
  std::uint64_t consumedCommandSequence_{0};
  bool receivedCommand_{false};

  rclcpp::Subscription<loader_sim_msgs::msg::VehicleCommand>::SharedPtr commandSubscription_;
  rclcpp_lifecycle::LifecyclePublisher<loader_sim_msgs::msg::VehicleState>::SharedPtr statePublisher_;

  std::array<double, 8> efforts_{};
  rclcpp::Time lastCommandTime_{0, 0, RCL_ROS_TIME};
  rclcpp::Time lastStatePublishTime_{0, 0, RCL_ROS_TIME};
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
}  // namespace loader_control

PLUGINLIB_EXPORT_CLASS(
    loader_control::LoaderCommandController,
    controller_interface::ControllerInterface)
