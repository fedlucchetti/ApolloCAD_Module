/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/timer_component.h"
#include "cyber/time/time.h"

#include "modules/common/util/util.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common/status/status.h"
#include "modules/common/monitor_log/monitor_log_buffer.h"


#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "modules/network/proto/network_conf.pb.h"
using namespace std;

#define MAXBUFF 80
#define PORT 9999
#define SA struct sockaddr


/**
 * @namespace apollo::network
 * @brief apollo::network
 */
namespace apollo {
namespace network {

/**
 * @class Network
 *
 * @brief network module main class, it processes localization, chassis, and
 * pad data to compute throttle, brake and steer values.
 */
class NetworkComponent final : public apollo::cyber::TimerComponent {
 public:
  NetworkComponent();
  /**
   * @brief obtain module name
   * @return module name
   */
  std::string Name() const;

 private:
  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init() override;

  /**
   * @brief module on_time function
   */
  bool Proc() override;


 private:
  localization::LocalizationEstimate latest_localization_;
  canbus::Chassis latest_chassis_;
  planning::ADCTrajectory latest_trajectory_;
  control::ControlCommand control_command_; 

  apollo::cyber::Time init_time_;

  
//   ControlConf control_conf_;

  std::mutex mutex_;

  std::shared_ptr<cyber::Reader<apollo::canbus::Chassis>> chassis_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;


   // Intercept Planning message
    std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>> planning_reader_;

    // Intercept Control message
    std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>> control_reader_;


    // Forward Planning message
    std::shared_ptr<cyber::Writer<apollo::planning::ADCTrajectory>> planning_writer_;

    // Forward Control message
    std::shared_ptr<cyber::Writer<apollo::control::ControlCommand>> control_writer_;

    // Monitor Network messages
    // std::shared_ptr<cyber::Writer<apollo::network::NetworkMessage>> network_writer_;


  common::monitor::MonitorLogBuffer monitor_logger_buffer_;

  string network_topic      = "/apollo/network";
  string planning_topic     = "/apollo/planning";
  string control_topic      = "/apollo/control";
  // string control_listen_topic = "/apollo/control_prepare";
    string control_listen_topic = "/apollo/control";

  string localization_topic = "/apollo/localization/pose";

  char* receive(int n);

  bool send(char* msg);

  vector<vector<double>> get_adcplanned_traj(
                            const auto & ptr_planning_msg_current);
    int n = 0;
    char buff[MAXBUFF];
		int sockfd, connfd;
		struct sockaddr_in servaddr, cli;	
    std::vector<std::string> keyWords{ "GET", "RELEASE",
                                     "DENY", "GRANT" };
};

CYBER_REGISTER_COMPONENT(NetworkComponent)
}  // namespace network
}  // namespace apollo
