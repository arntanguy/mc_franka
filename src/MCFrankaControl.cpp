/* Copyright 2020 mc_rtc development team */

#include "MCGlobalControllerBridge.h"
#include "PandaControlLoop.h"

// #include <mc_control/mc_global_controller.h>
#include "ControllerBridge.h"
#include "MCGlobalControllerBridge.h"
#ifdef WITH_MC_UDP
  #include "MCUDPControllerBridge.h"
#endif
  
#include <mc_panda/devices/Pump.h>
#include <mc_panda/devices/Robot.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

namespace mc_franka
{

// We want to modify this control interface to support both
// controlling the robot directly with MCGlobalController (existing) and controlling it through UPD
// The best way I see of achieving this is to provide a common interface to both the controller and the UDP communication
// and then have a specialization for each case


struct ControlLoopDataBase
{
  ControlLoopDataBase(ControlMode cm, bool show_network_warnings)
  : mode(cm), show_network_warnings(show_network_warnings), controller(nullptr), panda_threads(nullptr)
  {
  }
  ControlMode mode;
  bool show_network_warnings = false;
  // XXX should be either controller or UDP Client/Server
  //mc_control::MCGlobalController * controller;
  ControllerBridge * controller;
  std::thread * controller_run;
  std::condition_variable controller_run_cv;
  std::vector<std::thread> * panda_threads;
};

template<ControlMode cm, bool ShowNetworkWarnings>
struct ControlLoopData : public ControlLoopDataBase
{
  ControlLoopData() : ControlLoopDataBase(cm, ShowNetworkWarnings), pandas(nullptr) {}
  std::vector<PandaControlLoopPtr<cm, ShowNetworkWarnings>> * pandas;
};

template<ControlMode cm, bool ShowNetworkWarnings>
void * global_thread_init(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  auto loop_data = new ControlLoopData<cm, ShowNetworkWarnings>();
  // TODO choice between UDP and MCGlobalController
  loop_data->controller = new MCGlobalControllerBridge(gconfig);
  loop_data->panda_threads = new std::vector<std::thread>();
  auto & controller = *loop_data->controller;
  const double timeStep = controller.timeStep();
  if(timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_franka] mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(timeStep / 0.001);
  size_t freq = std::ceil(1 / timeStep);
  mc_rtc::log::info("[mc_franka] mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  // Initialize controlled panda robot
  loop_data->pandas = new std::vector<PandaControlLoopPtr<cm, ShowNetworkWarnings>>();
  auto & pandas = *loop_data->pandas;
  {
    std::vector<std::thread> panda_init_threads;
    std::mutex pandas_init_mutex;
    std::condition_variable pandas_init_cv;
    bool pandas_init_ready = false;
    // TODO rewrite such that it uses the robots in the config to create the panda control loops
    // It should only check if it matches with the MCGlobalController robots in MCGLobalControllerBridge

    if(auto frankaConfig = gconfig.config.find("Franka"))
    {
      if(auto robots = frankaConfig.find("robots"))
      {
        for(auto robot : *robots)
        {
          std::string ip = robot("ip");
          panda_init_threads.emplace_back([&, ip]() {
            {
              std::unique_lock<std::mutex> lock(pandas_init_mutex);
              pandas_init_cv.wait(lock, [&pandas_init_ready]() { return pandas_init_ready; });
            }
            // TODO TOMORROW HANDLE DEVICES
            auto pump = mc_panda::Pump::get(robot);
            auto & device = *mc_panda::Robot::get(robot);
            auto panda = std::unique_ptr<PandaControlLoop<cm, ShowNetworkWarnings>>(
                new PandaControlLoop<cm, ShowNetworkWarnings>(robot.name(), ip, n_steps, device, pump));
            device.addToLogger(controller.controller().logger(), robot.name());
            if(pump)
            {
              pump->addToLogger(controller.controller().logger(), robot.name());
            }
            std::unique_lock<std::mutex> lock(pandas_init_mutex);
            pandas.emplace_back(std::move(panda));
          });
        }
      }
    }

    // XXX some of this dead code should be moved to MCUDPControllerBridge
    for(auto & robot : robots)
    {
      if(robot.mb().nrDof() == 0)
      {
        continue;
      }
      if(std::find(ignoredRobots.begin(), ignoredRobots.end(), robot.name()) != ignoredRobots.end())
      {
        continue;
      }
      if(frankaConfig.has(robot.name()))
      {
        
      }
      else
      {
        mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                             robot.name());
      }
    }
    pandas_init_ready = true;
    pandas_init_cv.notify_all();
    for(auto & th : panda_init_threads)
    {
      th.join();
    }
  }
  for(auto & panda : pandas)
  {
    panda->init(controller);
  }
  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));
  // Start panda control loops
  static std::mutex startMutex;
  static std::condition_variable startCV;
  static bool startControl = false;
  for(auto & panda : pandas)
  {
    loop_data->panda_threads->emplace_back(
        [&]() { panda->controlThread(controller, startMutex, startCV, startControl, controller.running); });
  }
  startControl = true;
  startCV.notify_all();
  loop_data->controller_run = new std::thread([loop_data]() {
    auto controller_ptr = loop_data->controller;
    auto & controller = *controller_ptr;
    auto & pandas = *loop_data->pandas;
    std::mutex controller_run_mtx;
    timespec tv;
    clock_gettime(CLOCK_REALTIME, &tv);
    // Current time in milliseconds
    double current_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6;
    // Will record the time that passed between two runs
    double elapsed_t = 0;
    controller.controller().logger().addLogEntry("mc_franka_delay", [&elapsed_t]() { return elapsed_t; });
    while(controller.running)
    {
      std::unique_lock lck(controller_run_mtx);
      loop_data->controller_run_cv.wait(lck);
      clock_gettime(CLOCK_REALTIME, &tv);
      elapsed_t = tv.tv_sec * 1000 + tv.tv_nsec * 1e-6 - current_t;
      current_t = elapsed_t + current_t;
      // Update from panda sensors
      for(auto & panda : pandas)
      {
        panda->updateSensors(controller);
      }
      // Run the controller
      controller.run();
      // Update panda commands
      for(auto & panda : pandas)
      {
        panda->updateControl(controller);
      }
    }
  });
#ifndef WIN32
  // Lower thread priority so that it has a lesser priority than the real time
  // thread
  auto th_handle = loop_data->controller_run->native_handle();
  int policy = 0;
  sched_param param{};
  pthread_getschedparam(th_handle, &policy, &param);
  param.sched_priority = 99;
  if(pthread_setschedparam(th_handle, SCHED_RR, &param) != 0)
  {
    mc_rtc::log::warning(
        "[MCFrankaControl] Failed to lower calibration thread priority. If you are running on a real-time system, "
        "this might cause latency to the real-time loop.");
  }
#endif
  return loop_data;
}

template<ControlMode cm>
void * global_thread_init(mc_control::MCGlobalController::GlobalConfiguration & gconfig, bool ShowNetworkWarnings)
{
  if(ShowNetworkWarnings)
  {
    return global_thread_init<cm, true>(gconfig);
  }
  return global_thread_init<cm, false>(gconfig);
}

template<ControlMode cm, bool ShowNetworkWarnings>
void run_impl(void * data)
{
  auto control_data = static_cast<ControlLoopData<cm, ShowNetworkWarnings> *>(data);
  auto controller_ptr = control_data->controller;
  auto & controller = *controller_ptr;
  while(controller.running)
  {
    control_data->controller_run_cv.notify_one();
    // Sleep until the next cycle
    sched_yield();
  }
  for(auto & th : *control_data->panda_threads)
  {
    th.join();
  }
  control_data->controller_run->join();
  delete control_data->pandas;
  delete controller_ptr;
}

template<ControlMode cm>
void run_impl(void * data, bool ShowNetworkWarnings)
{
  if(ShowNetworkWarnings)
  {
    run_impl<cm, true>(data);
  }
  else
  {
    run_impl<cm, false>(data);
  }
}

/**
 * \brief Run the control loop
 *
 * \param data Pointer to the ControlLoopDataBase
 */
void run(void * data)
{
  auto control_data = static_cast<ControlLoopDataBase *>(data);
  switch(control_data->mode)
  {
    case ControlMode::Position:
      run_impl<ControlMode::Position>(data, control_data->show_network_warnings);
      break;
    case ControlMode::Velocity:
      run_impl<ControlMode::Velocity>(data, control_data->show_network_warnings);
      break;
    case ControlMode::Torque:
      run_impl<ControlMode::Torque>(data, control_data->show_network_warnings);
      break;
  }
}

void * init(int argc, char * argv[], uint64_t & cycle_ns)
{
  std::string conf_file = "";
  po::options_description desc("MCFrankaControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    std::cout << "see etc/sample.yaml for libfranka configuration\n";
    return nullptr;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("Franka"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Franka section in the configuration, see etc/sample.yaml for an example");
  }
  auto frankaConfig = gconfig.config("Franka");
  ControlMode cm = frankaConfig("ControlMode", ControlMode::Velocity);
  bool ShowNetworkWarnings = frankaConfig("ShowNetworkWarnings", true);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        return global_thread_init<ControlMode::Position>(gconfig, ShowNetworkWarnings);
      case ControlMode::Velocity:
        return global_thread_init<ControlMode::Velocity>(gconfig, ShowNetworkWarnings);
      case ControlMode::Torque:
        return global_thread_init<ControlMode::Torque>(gconfig, ShowNetworkWarnings);
      default:
        return nullptr;
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return nullptr;
  }
}

} // namespace mc_franka
