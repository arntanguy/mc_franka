/* Copyright 2020 mc_rtc development team */

#pragma once

#include "PandaControlType.h"
#include "defs.h"

#include <franka/log.h>
#include <franka/robot.h>
#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/Robot.h>

#include <mc_control/mc_global_controller.h>

#include <condition_variable>
#include <cstring>
#include <thread>

namespace mc_franka
{

// control thread
// sensors: mc_franka::RobotState
// control: mc_franka::RobotCommand

/** The PandaControlLoop implements two things:
 *
 * 1. Update mc_rtc sensor information from the panda state provided by
 * libfranka
 *
 * 2. Use mc_rtc output to send relevant control information to the panda
 */
template<ControlMode cm, bool ShowNetworkWarnings>
struct PandaControlLoop
{
  /** Constructor
   *
   * \param name Identifier of the robot
   *
   * \param ip IP of the robot
   *
   * \param steps How often the robot should get updated, 1 means every step, 2
   * means every 2 steps...
   *
   * \param device PandaDevice associated to the robot
   *
   * \param pump Pump associated to the robot (nullptr if none)
   */
  PandaControlLoop(const std::string & name,
                   const std::string & ip,
                   size_t steps,
                   mc_panda::Robot & device,
                   mc_panda::Pump * pump);

  /** Initialize mc_rtc robot from the current state */
  void init(mc_control::MCGlobalController & controller);

  /** Update sensors in mc_rtc instance */
  void updateSensors(const franka::RobotState & state, mc_control::MCGlobalController & controller);

  /** Get the latest sensor values
   */
  franka::RobotState getSensors()
  {
    std::unique_lock<std::mutex> senLock(updateSensorsMutex_);
    return state_;
  }

  /** Update command from mc_rtc output
   * This is expected to be called from mc_rtc's control thread
   */
  void updateCommand(mc_control::MCGlobalController & controller, franka::RobotCommand & command);

  franka::RobotCommand getCommand()
  {
    std::unique_lock<std::mutex> ctlLock(updateControlMutex_);
    return command_;
  }

  /**
   * Update command from mc_rtc output
   * This is expected to be called from the RT thread at robot's control frequency
   */
  void updateControl(const franka::RobotCommand & command);

  /** Start the libfranka control loop
   *
   * The start is synchronized with other panda robots
   *
   * \param controller Controller that is running this loop
   *
   * \param startM Mutex to the start variable
   *
   * \param startCV Condition variable used to trigger start
   *
   * \param start Monitoring value
   *
   * \param running True while mc_rtc is running
   */
  void controlThread(mc_control::MCGlobalController & controller,
                     std::mutex & startM,
                     std::condition_variable & startCV,
                     bool & start,
                     bool & running);

private:
  std::string name_; // Name of the mc_rtc robot being controlled
  franka::Robot robot_; // Real robot control interface
  franka::RobotState state_; // Last state from robot
  PandaControlType<cm> control_;
  mc_panda::Robot & device_;
  size_t steps_ = 1;
  mc_rtc::Logger logger_;
  size_t sensor_id_ = 0;
  // rbd::MultiBodyConfig command_;
  franka::RobotCommand command_; // Desired command computed by mc_rtc
  size_t control_id_ = 0;
  size_t prev_control_id_ = 0;
  double delay_ = 0;

  mutable std::mutex updateSensorsMutex_;
  mutable std::mutex updateControlMutex_;

  std::vector<double> sensorsBuffer_ = std::vector<double>(7, 0.0);
};

template<ControlMode cm, bool ShowNetworkWarnings>
using PandaControlLoopPtr = std::unique_ptr<PandaControlLoop<cm, ShowNetworkWarnings>>;

template<ControlMode cm, bool ShowNetworkWarnings>
PandaControlLoop<cm, ShowNetworkWarnings>::PandaControlLoop(const std::string & name,
                                                            const std::string & ip,
                                                            size_t steps,
                                                            mc_panda::Robot & device,
                                                            mc_panda::Pump * pump)
: name_(name), robot_(ip, franka::RealtimeConfig::kIgnore), state_(robot_.readOnce()), control_(state_),
  device_(device), steps_(steps), logger_(mc_rtc::Logger::Policy::THREADED, "/tmp", "mc-franka-" + name_)
{
  robot_.automaticErrorRecovery();
  static auto panda_init_t = clock::now();
  auto now = clock::now();
  duration_us dt = now - panda_init_t;
  mc_rtc::log::info("[mc_franka] Elapsed time since the creation of another PandaControlLoop: {}us", dt.count());
  if(pump)
  {
    pump->connect(ip);
    pump->addToLogger(logger_, name);
  }
  device.connect(&robot_);
  device.addToLogger(logger_, name);
}

template<ControlMode cm, bool ShowNetworkWarnings>
void PandaControlLoop<cm, ShowNetworkWarnings>::init(mc_control::MCGlobalController & controller)
{
  logger_.start(controller.current_controller(), 0.001);
  logger_.addLogEntry("sensors_id", [this]() { return sensor_id_; });
  logger_.addLogEntry("prev_control_id", [this]() { return prev_control_id_; });
  logger_.addLogEntry("control_id", [this]() { return control_id_; });
  logger_.addLogEntry("delay", [this]() { return delay_; });
  updateSensors(state_, controller); // Read initial sensor values
  auto command = franka::RobotCommand{};
  updateCommand(controller, command); // Use sensor values as initial command
  updateControl(command);
  prev_control_id_ = control_id_;
  auto & robot = controller.controller().robots().robot(name_);
  auto & real = controller.controller().realRobots().robot(name_);
  const auto & rjo = robot.refJointOrder();
  for(size_t i = 0; i < rjo.size(); ++i)
  {
    auto jIndex = robot.jointIndexByName(rjo[i]);
    robot.mbc().q[jIndex][0] = state_.q[i];
    robot.mbc().jointTorque[jIndex][0] = state_.tau_J[i];
  }
  robot.forwardKinematics();
  real.mbc() = robot.mbc();
}

template<ControlMode cm, bool ShowNetworkWarnings>
void PandaControlLoop<cm, ShowNetworkWarnings>::updateSensors(const franka::RobotState & state,
                                                              mc_control::MCGlobalController & controller)
{
  auto & robot = controller.controller().robots().robot(name_);
  using GC = mc_control::MCGlobalController;
  using set_sensor_t = void (GC::*)(const std::string &, const std::vector<double> &);
  auto updateSensor = [&controller, &robot, this](set_sensor_t set_sensor, const std::array<double, 7> & data) {
    assert(sensorsBuffer_.size() == 7);
    std::memcpy(sensorsBuffer_.data(), data.data(), 7 * sizeof(double));
    (controller.*set_sensor)(robot.name(), sensorsBuffer_);
  };
  updateSensor(&GC::setEncoderValues, state.q);
  updateSensor(&GC::setEncoderVelocities, state.dq);
  updateSensor(&GC::setJointTorques, state.tau_J);
  auto wrench = sva::ForceVecd::Zero();
  wrench.force().x() = state.K_F_ext_hat_K[0];
  wrench.force().y() = state.K_F_ext_hat_K[1];
  wrench.force().z() = state.K_F_ext_hat_K[2];
  wrench.couple().x() = state.K_F_ext_hat_K[3];
  wrench.couple().y() = state.K_F_ext_hat_K[4];
  wrench.couple().z() = state.K_F_ext_hat_K[5];
  robot.data()->forceSensors[robot.data()->forceSensorsIndex.at("LeftHandForceSensor")].wrench(wrench);
  device_.state(state);
}

template<ControlMode cm, bool ShowNetworkWarnings>
void PandaControlLoop<cm, ShowNetworkWarnings>::updateCommand(mc_control::MCGlobalController & controller,
                                                              franka::RobotCommand & command)
{
  const auto & robot = controller.controller().robots().robot(name_);
  for(size_t i = 0; i < robot.refJointOrder().size(); ++i)
  {
    auto jIndex = robot.jointIndexByName(robot.refJointOrder()[i]);
    command.joint_positions.q[i] = robot.mbc().q[jIndex][0];
    command.joint_velocities.dq[i] = robot.mbc().alpha[jIndex][0];
    command.torques.tau_J[i] = robot.mbc().jointTorque[jIndex][0];
  }
}

template<ControlMode cm, bool ShowNetworkWarnings>
void PandaControlLoop<cm, ShowNetworkWarnings>::updateControl(const franka::RobotCommand & command)
{
  std::unique_lock<std::mutex> lock(updateControlMutex_);
  command_ = command;
  control_id_++;
}

template<ControlMode cm, bool ShowNetworkWarnings>
void PandaControlLoop<cm, ShowNetworkWarnings>::controlThread(mc_control::MCGlobalController & controller,
                                                              std::mutex & startM,
                                                              std::condition_variable & startCV,
                                                              bool & start,
                                                              bool & running)
{
  {
    std::unique_lock<std::mutex> lock(startM);
    startCV.wait(lock, [&]() { return start; });
  }
  auto start_t = clock::now();
  control_.control(
      robot_,
      [&, this](const franka::RobotState & stateIn, franka::Duration dt) -> typename PandaControlType<cm>::ReturnT {
        std::unique_lock<std::mutex> ctlLock(updateControlMutex_);
        std::unique_lock<std::mutex> senLock(updateSensorsMutex_);
        auto now = clock::now();
        delay_ = duration_ms(now - start_t).count();
        start_t = now;
        state_ = stateIn;
        sensor_id_ += dt.toMSec();
        if(sensor_id_ % steps_ == 0)
        {
          if(ShowNetworkWarnings && control_id_ != prev_control_id_ + dt.toMSec())
          {
            mc_rtc::log::warning("[mc_franka] {} missed control data (previous: {}, current: {}, expected: {}", name_,
                                 prev_control_id_, control_id_, prev_control_id_ + dt.toMSec());
          }
          prev_control_id_ = control_id_;
        }
        if(running)
        {
          logger_.log();
          auto & robot = controller.controller().robots().robot(name_);
          return control_.update(robot, command_, sensor_id_ % steps_, steps_);
        }
        return franka::MotionFinished(control_);
      });
}

} // namespace mc_franka
