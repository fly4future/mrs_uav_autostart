/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/coro/task.hpp>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/errorgraph/error_publisher.h>
#include <mrs_lib/node.h>

#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/gazebo_spawner_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/general_robot_info.hpp>

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_autostart
{

namespace automatic_start
{

/* class AutomaticStart //{ */

// state machine
typedef enum
{
  STATE_IDLE,
  STATE_TAKEOFF,
  STATE_FINISHED
} LandingStates_t;

const char *state_names[3] = {"IDLING", "TAKEOFF", "FINISHED"};

class AutomaticStart : public mrs_lib::Node {

public:
  AutomaticStart(rclcpp::NodeOptions options);

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_;

  std::atomic<bool> is_initialized_ = false;

  std::string _uav_name_;
  bool        _simulation_;

  std::shared_ptr<mrs_lib::errorgraph::ErrorPublisher> error_publisher_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_toggle_control_output_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool> service_client_arm_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> service_client_takeoff_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>               sh_hw_api_status_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics>  sh_gazebo_spawner_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>          sh_general_robot_info_;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<std_msgs::msg::Bool> ph_ready_to_enable_control_output_;

  // | ----------------------- main timer ----------------------- |

  std::shared_ptr<TimerType> timer_main_;
  mrs_lib::Task<>            timerMain();
  double                     _main_timer_rate_;

  // | ------------------------- hw api ------------------------- |

  void              callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg);
  std::atomic<bool> hw_api_connected_ = false;
  std::mutex        mutex_hw_api_status_;

  // | --------------- Gazebo spawner diagnostics --------------- |

  void                                    callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool>                       got_gazebo_spawner_diagnostics = false;
  mrs_msgs::msg::GazeboSpawnerDiagnostics gazebo_spawner_diagnostics_;
  std::mutex                              mutex_gazebo_spawner_diagnostics_;

  // | ----------------- arm and offboard check ----------------- |

  rclcpp::Time armed_time_;
  bool         armed_ = false;

  rclcpp::Time offboard_time_;
  bool         offboard_ = false;

  bool we_toggled_output_ = false;

  // | ------------------------ routines ------------------------ |

  mrs_lib::Task<bool> takeoff();

  mrs_lib::Task<bool> toggleControlOutput(const bool &value);
  mrs_lib::Task<bool> disarm();

  bool isGazeboSimulation(void);

  bool is_gazebo_simulation_ = false;

  // | ---------------------- other params ---------------------- |

  double      _pre_takeoff_sleep_;
  bool        _handle_takeoff_ = false;
  double      _safety_timeout_;
  double      _control_output_timeout_;

  // | ---------------------- state machine --------------------- |

  uint                current_state = STATE_IDLE;
  mrs_lib::Task<void> changeState(LandingStates_t new_state);
};

//}

/* AutomaticStart() //{ */

AutomaticStart::AutomaticStart(rclcpp::NodeOptions options) : Node("automatic_start", options) {

  node_  = this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_          = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  error_publisher_ = std::make_shared<mrs_lib::errorgraph::ErrorPublisher>(node_, clock_, "AutomaticStart", "main");

  armed_      = false;
  armed_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  offboard_      = false;
  offboard_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  mrs_lib::ParamLoader param_loader(node_, "AutomaticStart");

  std::string custom_config_path;

  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    if (!param_loader.addYamlFile(custom_config_path)) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load custom_config");
      error_publisher_->addOneshotError("failed to load custom_config");
      error_publisher_->flushAndShutdown();
    }
  }

  if (!param_loader.addYamlFileFromParam("config_private")) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load config_private");
    error_publisher_->addOneshotError("failed to load config_private");
    error_publisher_->flushAndShutdown();
  }

  if (!param_loader.addYamlFileFromParam("config_public")) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load config_public");
    error_publisher_->addOneshotError("failed to load config_public");
    error_publisher_->flushAndShutdown();
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("simulation", _simulation_);

  param_loader.loadParam("mrs_uav_autostart/main_timer_rate", _main_timer_rate_);
  param_loader.loadParam("mrs_uav_autostart/control_output_timeout", _control_output_timeout_);

  param_loader.loadParam("mrs_uav_autostart/safety_timeout", _safety_timeout_);
  param_loader.loadParam("mrs_uav_autostart/pre_takeoff_sleep", _pre_takeoff_sleep_);

  param_loader.loadParam("mrs_uav_autostart/handle_takeoff", _handle_takeoff_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(this_node().get_logger(), "Could not load all parameters!");
    error_publisher_->addOneshotError("Could not load all parameters!");
    error_publisher_->flushAndShutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_;

  sh_hw_api_status_        = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(shopts, "~/hw_api_status_in", &AutomaticStart::callbackHwApiStatus, this);
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_gazebo_spawner_diag_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::GazeboSpawnerDiagnostics>(shopts, "~/gazebo_spawner_diagnostics_in",
                                                                                                 &AutomaticStart::callbackGazeboSpawnerDiagnostics, this);
  sh_general_robot_info_   = mrs_lib::SubscriberHandler<mrs_msgs::msg::GeneralRobotInfo>(shopts, "~/general_robot_info_in");

  // | ----------------------- publishers ----------------------- |

  ph_ready_to_enable_control_output_ = mrs_lib::PublisherHandler<std_msgs::msg::Bool>(node_, "~/ready_to_enable_control_output_out");

  // | --------------------- service clients -------------------- |

  service_client_takeoff_               = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/takeoff_out", cbkgrp_);
  service_client_toggle_control_output_ = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/toggle_control_output_out", cbkgrp_);
  service_client_arm_                   = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "~/arm_out", cbkgrp_);

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node           = node_;
  timer_opts_start.autostart      = true;
  timer_opts_start.callback_group = cbkgrp_;

  timer_main_ = std::make_shared<TimerType>(timer_opts_start, rclcpp::Rate(_main_timer_rate_, clock_), &AutomaticStart::timerMain, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "initialized");
}

//}

// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* callbackHwApiStatus() //{ */

void AutomaticStart::callbackHwApiStatus(const mrs_msgs::msg::HwApiStatus::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting HW API status");

  std::scoped_lock lock(mutex_hw_api_status_);

  // check armed_ state
  if (armed_ == false) {

    // if armed_ state changed to true, please "start the clock"
    if (msg->armed) {

      armed_      = true;
      armed_time_ = clock_->now();
    }

    // if we were armed_ previously
  } else if (armed_ == true) {

    // and we are not really now
    if (!msg->armed) {

      armed_ = false;
    }
  }

  // check offboard_ state
  if (offboard_ == false) {

    // if offboard_ state changed to true, please "start the clock"
    if (msg->offboard) {

      offboard_      = true;
      offboard_time_ = clock_->now();
    }

    // if we were in offboard_ previously
  } else if (offboard_ == true) {

    // and we are not really now
    if (!msg->offboard) {

      offboard_ = false;
    }
  }

  if (msg->connected) {
    hw_api_connected_ = true;
  }
}

//}

/* callbackGazeboSpawnerDiagnostics() //{ */

void AutomaticStart::callbackGazeboSpawnerDiagnostics(const mrs_msgs::msg::GazeboSpawnerDiagnostics::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting spawner diagnostics");

  {
    std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

    gazebo_spawner_diagnostics_ = *msg;

    got_gazebo_spawner_diagnostics = true;
  }
}

//}

// --------------------------------------------------------------
// |                           timers                           |
// --------------------------------------------------------------

/* timerMain() //{ */

mrs_lib::Task<> AutomaticStart::timerMain() {

  if (!is_initialized_) {
    co_return;
  }

  bool got_control_manager_diag = sh_control_manager_diag_.hasMsg();
  bool got_hw_api               = sh_hw_api_status_.hasMsg() && hw_api_connected_;
  bool got_general_robot_info   = sh_general_robot_info_.hasMsg();

  if (!got_control_manager_diag || !got_hw_api || !got_general_robot_info) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 5000, "waiting for data: ControlManager=%s, HW Api=%s, DiagnosticsManager=%s",
                         got_control_manager_diag ? "true" : "FALSE", got_hw_api ? "true" : "FALSE", got_general_robot_info ? "true" : "FALSE");
    if (!got_hw_api) {
      error_publisher_->addWaitingForNodeError({"HwApiManager", "main"});
    }
    if (!got_control_manager_diag) {
      error_publisher_->addWaitingForNodeError({"ControlManager", "main"});
    }
    if (!got_general_robot_info) {
      error_publisher_->addWaitingForNodeError({"DiagnosticsManager", "main"});
    }

    co_return;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_hw_api_status_, armed_, offboard_, armed_time_, offboard_time_);
  auto control_manager_diagnostics                  = sh_control_manager_diag_.getMsg();

  switch (current_state) {

  case STATE_IDLE: {

    // | --------------------- preflight check -------------------- |

    const auto &preflight = sh_general_robot_info_.getMsg()->preflight_status;

    bool possibly_in_the_air = !(preflight.speed_ok && preflight.height_ok && preflight.gyro_ok);

    if (!offboard && possibly_in_the_air) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "preflight check failed, the UAV is possibly in the air");

      if (armed) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                             "-- the UAV is also armed!! finishing to prevent "
                             "unwanted system activation");

        if (we_toggled_output_) {

          bool res = co_await toggleControlOutput(false);

          if (!res) {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output OFF");
          }
        }

        co_await changeState(STATE_FINISHED);

        co_return;
      }

      co_return;
    }

    // | -------------------- ready to takeoff -------------------- |

    bool control_output_enabled = sh_control_manager_diag_.getMsg()->output_enabled;

    std_msgs::msg::Bool ready_to_enable_control_output_msg;
    ready_to_enable_control_output_msg.data = false;

    // | -------------------- preflight checks -------------------- |

    bool ready_to_enable_control_output = preflight.topics_ok && preflight.position_valid;

    // | ---------------------------------------------------------- |

    ready_to_enable_control_output_msg.data = ready_to_enable_control_output;
    ph_ready_to_enable_control_output_.publish(ready_to_enable_control_output_msg);

    if (armed && !control_output_enabled) {

      if (ready_to_enable_control_output) {

        bool res = co_await toggleControlOutput(true);

        if (!res) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON");
        } else {
          we_toggled_output_ = true;
        }
      }

      double time_from_arming = (clock_->now() - armed_time).seconds();

      if (armed_time.seconds() > 0 && time_from_arming > _control_output_timeout_) {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not set control output ON for %.2f secs, disarming", _control_output_timeout_);
        co_await disarm();
        co_await changeState(STATE_FINISHED);
      }
    }

    if (_simulation_ && isGazeboSimulation()) {

      std::scoped_lock lock(mutex_gazebo_spawner_diagnostics_);

      if (got_gazebo_spawner_diagnostics) {

        if (!gazebo_spawner_diagnostics_.spawn_called || gazebo_spawner_diagnostics_.processing) {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) waiting for spawner to finish spawning UAVs");
          co_return;
        }

      } else {

        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "(simulation) missing spawner diagnostics");
        co_return;
      }
    }

    // when armed and in offboard, takeoff
    if (armed && offboard && control_output_enabled) {

      if (!_handle_takeoff_) {
        co_await changeState(STATE_FINISHED);
      } else {

        rclcpp::Duration armed_time_diff    = clock_->now() - armed_time;
        rclcpp::Duration offboard_time_diff = clock_->now() - offboard_time;

        if (armed_time_diff.seconds() > _safety_timeout_ && offboard_time_diff.seconds() > _safety_timeout_) {

          co_await changeState(STATE_TAKEOFF);

        } else {

          double min = (armed_time_diff < offboard_time_diff) ? armed_time_diff.seconds() : offboard_time_diff.seconds();

          RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "taking off in %.0f", (_safety_timeout_ - min));
        }
      }
    }

    break;
  }

  case STATE_TAKEOFF: {

    // if takeoff finished
    if (control_manager_diagnostics->flying_normally) {

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "takeoff finished");

      co_await changeState(STATE_FINISHED);

    } else {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "waiting for the takeoff to finish");
    }

    break;
  }

  case STATE_FINISHED: {

    RCLCPP_INFO_ONCE(node_->get_logger(), "finished");

    timer_main_->stop();

    break;
  }
  }
}

//}

// --------------------------------------------------------------
// |                          routines                          |
// --------------------------------------------------------------

/* changeState() //{ */

mrs_lib::Task<> AutomaticStart::changeState(LandingStates_t new_state) {

  RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "switching states %s -> %s", state_names[current_state], state_names[new_state]);

  switch (new_state) {

  case STATE_IDLE: {

    break;
  }

  case STATE_TAKEOFF: {

    if (_pre_takeoff_sleep_ > 1.0) {
      RCLCPP_INFO(node_->get_logger(), "sleeping for %.2f secs before takeoff", _pre_takeoff_sleep_);
      clock_->sleep_for(std::chrono::duration<double>(_pre_takeoff_sleep_));
    }

    bool res = co_await takeoff();

    if (!res) {

      current_state = STATE_FINISHED;

      co_return;
    }

    break;
  }

  case STATE_FINISHED: {

    break;
  }

  break;
  }

  current_state = new_state;
}

//}

/* takeoff() //{ */

mrs_lib::Task<bool> AutomaticStart::takeoff() {

  RCLCPP_INFO(node_->get_logger(), "taking off");

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  auto response = co_await service_client_takeoff_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "taking off failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for taking off failed");
  }

  co_return false;
}

//}

/* toggleControlOutput() //{ */

mrs_lib::Task<bool> AutomaticStart::toggleControlOutput(const bool &value) {

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "setting control output %s", value ? "ON" : "OFF");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = value;

  auto response = co_await service_client_toggle_control_output_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "setting of control output failed: %s", response.value()->message.c_str());
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for toggling control output failed");
  }

  co_return false;
}

//}

/* disarm() //{ */

mrs_lib::Task<bool> AutomaticStart::disarm() {

  if (!hw_api_connected_) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "cannot disarm, missing HW API status!");

    co_return false;
  }

  auto [armed, offboard, armed_time, offboard_time] = mrs_lib::get_mutexed(mutex_hw_api_status_, armed_, offboard_, armed_time_, offboard_time_);

  if (offboard) {

    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "cannot disarm, already in offboard mode!");

    co_return false;
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "disarming");

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = false;

  auto response = co_await service_client_arm_.callAwaitable(request);

  if (response) {

    if (response.value()->success) {

      co_return true;

    } else {

      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "disarming failed");
    }

  } else {

    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "service call for disarming failed");
  }

  co_return false;
}

//}

/* isGazeboSimulation() //{ */

bool AutomaticStart::isGazeboSimulation(void) {

  if (is_gazebo_simulation_) {
    return true;
  }

  for (auto &node : node_->get_node_names()) {
    if (node.find("mrs_drone_spawner") != std::string::npos) {
      RCLCPP_INFO(node_->get_logger(), "MRS Gazebo Simulation detected");
      is_gazebo_simulation_ = true;
      return true;
    }
  }

  return false;
}

//}

} // namespace automatic_start

} // namespace mrs_uav_autostart

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_uav_autostart::automatic_start::AutomaticStart)
