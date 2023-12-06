/* Copyright 2020 mc_rtc development team */

#pragma once

#include <mc_rbdyn/Robot.h>

#include <franka/exception.h>
#include <franka/robot.h>
#include <mc_rtc/io_utils.h>

#include "ControlMode.h"

namespace mc_franka
{

/** PandaControlType implements a control callback for libfranka
 *
 * Depending on ControlMode the callback will provide position, velocity or
 * torques commands to the robot
 */
template<ControlMode cm>
struct PandaControlType
{
  static_assert(static_cast<int>(cm) == static_cast<int>(cm) + 1, "This must be specialized");
};

template<>
struct PandaControlType<ControlMode::Position> : public franka::JointPositions
{
  using ReturnT = franka::JointPositions;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointPositions(state.q), prev_q_(state.q) {}

  // Interpolate control value from the data in a robot
  franka::JointPositions update(const mc_rbdyn::Robot & robot,
                                const franka::RobotCommand & command,
                                size_t iter,
                                size_t N)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < q.size(); ++i)
    {
      q[i] = prev_q_[i] + (iter + 1) * (command.joint_positions.q[i] - prev_q_[i]) / N;
    }
    if(iter + 1 == N)
    {
      prev_q_ = q;
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 100);
  }

private:
  std::array<double, 7> prev_q_;
};

template<>
struct PandaControlType<ControlMode::Velocity> : public franka::JointVelocities
{
  using ReturnT = franka::JointVelocities;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointVelocities(state.dq) {}

  // Update control value from the data in a robot
  franka::JointVelocities update(const mc_rbdyn::Robot & robot, const franka::RobotCommand & command, size_t, size_t)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < dq.size(); ++i)
    {
      dq[i] = command.joint_velocities.dq[i];
    }
    mc_rtc::log::info("Velocity would have been: {}", mc_rtc::io::to_string(dq));

    // XXX do not move for now
    /* for(size_t i = 0; i < dq.size(); ++i) */
    /* { */
    /*   dq[i] = 0; */
    /* } */

    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 1000);
  }
};

template<>
struct PandaControlType<ControlMode::Torque> : public franka::Torques
{
  using ReturnT = franka::Torques;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::Torques(state.tau_J) {}

  // Update control value from the data in a robot
  franka::Torques update(const mc_rbdyn::Robot & robot, const franka::RobotCommand & command, size_t, size_t)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < tau_J.size(); ++i)
    {
      tau_J[i] = command.torques.tau_J[i];
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, true, 1000);
  }
};

} // namespace mc_franka
