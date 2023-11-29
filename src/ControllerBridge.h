#pragma once
#include <vector>

namespace mc_franka
{

struct ControllerBridge
{
  virtual void setEncoderValues(const std::vector<double> & values) = 0;
  virtual void setEncoderVelocities(const std::vector<double> & values) = 0;
  virtual void setJointTorques(const std::vector<double> & values) = 0;
  virtual std::vector<double> getEncoderValues() const noexcept = 0;
  virtual std::vector<double> getEncoderVelocities() const noexcept = 0;
  virtual std::vector<double> getJointTorques() const noexcept = 0;
  virtual double timeStep() const = 0;
};

}

