#pragma once
#include "ControllerBridge.h"
#include <mc_control/mc_global_controller.h>

namespace mc_franka
{
struct MCGlobalControllerBridge : public ControllerBridge
{
  MCGlobalControllerBridge(mc_control::MCGlobalController::GlobalConfiguration gconfig) : controller(new mc_control::MCGlobalController(gconfig))
  {
    auto & robots = controller->controller().robots();
    // Initialize all real robots
    // XXX why is the interface doing this here? I don't think this is needed.
    for(size_t i = controller->realRobots().size(); i < robots.size(); ++i)
    {
      controller->realRobots().robotCopy(robots.robot(i), robots.robot(i).name());
    }
  }

  // implement all the functions of ControllerBridge
  void setEncoderValues(const std::vector<double> & values) override
  {
    controller->setEncoderValues(values);
  }
  void setEncoderVelocities(const std::vector<double> & values) override
  {
    controller->setEncoderVelocities(values);
  }
  void setJointTorques(const std::vector<double> & values) override
  {
    controller->setJointTorques(values);
  }
  std::vector<double> getEncoderValues() const noexcept override
  {
    return controller->controller().robot().encoderValues();
  }
  std::vector<double> getEncoderVelocities() const noexcept override
  {
    return controller->controller().robot().encoderVelocities();
  }
  std::vector<double> getJointTorques() const noexcept override
  {
    return controller->controller().robot().jointTorques();
  }
  double timeStep() const override
  {
    return controller->controller().timeStep;
  }

  protected:
    mc_control::MCGlobalController * controller;
};
}
