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
#include "modules/network/network_component.h"

 #include <iostream>


#include "absl/strings/str_cat.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/latency_recorder/latency_recorder.h"

namespace apollo {
namespace network {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::control::ControlCommand;


std::string NetworkComponent::Name() const { return "network"; }

NetworkComponent::NetworkComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::MONITOR) {}

bool NetworkComponent::Init() {
  init_time_ = Clock::Now();
  fprintf(stderr, "Init() \n");

  
  planning_reader_ =
      node_->CreateReader<ADCTrajectory>(
          planning_topic, nullptr);
  ACHECK(planning_reader_ != nullptr);


  localization_reader_ = 
      node_->CreateReader<LocalizationEstimate>(
          localization_topic, nullptr);
  ACHECK(localization_reader_ != nullptr);


  control_reader_ =
      node_->CreateReader<ControlCommand>(
          control_listen_topic, nullptr);  
  ACHECK(control_reader_ != nullptr);

  control_writer_ = node_->CreateWriter<ControlCommand>(control_topic);


  // Create/accept/connect socket here

  fprintf(stderr, "TCP connection established \n");

  return true;
}


bool NetworkComponent::Proc() {
  fprintf(stderr, "Proc() \n");
  const auto start_time = Clock::Now();
  planning_reader_->Observe();
  const auto &trajectory_msg   = planning_reader_->GetLatestObserved();
  if (trajectory_msg == nullptr) {
    AERROR << "trajectory msg is not ready!";
    return false;
  }
  localization_reader_->Observe();
  const auto &localization_msg = localization_reader_->GetLatestObserved();
  if (localization_msg == nullptr) {
    AERROR << "localization msg is not ready!";
    return false;
  }

  control_reader_->Observe();
  const auto &control_msg      = control_reader_->GetLatestObserved();
  if (control_msg == nullptr) {
    AERROR << "control msg is not ready!";
    return false;
  }

  const auto end_time = Clock::Now();
  const double time_diff_ms = (end_time - start_time).ToSecond() * 1e3;
  double x_current = localization_msg->pose().position().x();
  double y_current = localization_msg->pose().position().y();


  // double throttle = control_msg->throttle();
  fprintf(stderr, "Setting throttle to 100% \n");
  control_msg->set_brake(100);
  fprintf(stderr, "Actuating throttle to 100% \n");
  control_writer_->Write(control_msg);
  fprintf(stderr, "DONE \n");


  // vector<vector<double>> planned_trajectory = get_adcplanned_traj(trajectory_msg);


  // AINFO << "current x :" << x_current << "\n";
  // AINFO << "current y :" << y_current << "\n";

  fprintf(stderr, "%f current x \n",x_current);
  fprintf(stderr, "%f current y \n",y_current);
  fprintf(stderr, "%f time_diff_ms \n",time_diff_ms);
  system ("clear");

  return true;
}



char* NetworkComponent::receive(int n){
  std::string response;
  bzero(buff, MAXBUFF);
  read(connfd, buff, MAXBUFF);
  std::string received_data(buff);
  std::string expected_keyword = keyWords[n];
  printf("Correct word: %s\n", expected_keyword);
  printf("From client: %s\n", received_data.c_str());
  if (received_data == expected_keyword) {
    response = "OK";
  }
  else {
    response = "NOT OK";
  }
  char* finalResponse = new char[response.size() + 1];
  strcpy(finalResponse, response.c_str());
  return finalResponse;
}

bool NetworkComponent::send(char* response){
  printf("To client: %s\n", response);
  write(connfd, response, sizeof(response));
  bzero(response, sizeof(response));
  return 1;
}


vector<vector<double>> NetworkComponent::get_adcplanned_traj(
                            const auto & ptr_planning_msg_current){
  // Useful function to convert the planned trajectory with planned velocity 
  // into a 5D vector [time,x,y,z,speed]
  vector<vector<double>> adc_trajectory; //
  vector<double> current_adc_traj(5);     // [time,x,y,z,v]
  if(ptr_planning_msg_current!=nullptr){
    for (auto const& trajectory_point:  ptr_planning_msg_current->trajectory_point()) {
      current_adc_traj.at(0)=trajectory_point.relative_time();
      current_adc_traj.at(1)=trajectory_point.path_point().x();
      current_adc_traj.at(2)=trajectory_point.path_point().y();
      current_adc_traj.at(3)=trajectory_point.path_point().z();
      current_adc_traj.at(4)=trajectory_point.v();
      adc_trajectory.push_back(current_adc_traj);
      }
  }
  return adc_trajectory;
}

}  // namespace network
}  // namespace apollo
