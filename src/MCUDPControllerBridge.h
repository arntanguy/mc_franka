#pragma once
#include "ControllerBridge.h"

namespace mc_franka
{
struct MCUDPControllerBridge : public ControllerBridge
{
  // implement all the functions of ControllerBridge
  void setEncoderValues(const std::vector<double> & values) override
  {
    // XXX
  }
  void setEncoderVelocities(const std::vector<double> & values) override
  {
    // XXX
  }
  void setJointTorques(const std::vector<double> & values) override
  {
    // XXX
  }
  std::vector<double> getEncoderValues() const noexcept override
  {
    // XXX
    return {};
  }
  std::vector<double> getEncoderVelocities() const noexcept override
  {
    // XXX
    return {};
  }
  std::vector<double> getJointTorques() const noexcept override
  {
    // XXX
    return {};
  }
  double timeStep() const override
  {
    // XXX
    return 0;
  }
};

}
