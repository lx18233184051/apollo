/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * limitations under the LicensLe.
 *****************************************************************************/
#include "modules/latency/latency.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace latency {
using apollo::planning::ADCTrajectory;
using apollo::relative_map::MapMsg;
using apollo::routing::RoutingResponse;

bool LatencyComponent::Init() {
  rerouting_writer_ =
      node_->CreateWriter<RoutingRequest>("/apollo/routing_request");
  routing_reader_ = node_->CreateReader<RoutingResponse>(
      "/apollo/routing_response",
      [this](const std::shared_ptr<RoutingResponse>& routing) {
        Header* header = routing_latency_.add_header();
        header->CopyFrom(Transform(routing->header()));
        AERROR << "Received routing data: run routing callback."
               << routing->header().DebugString();
      });
  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
      "/apollo/perception/traffic_light",
      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
        Header* header = traffic_light_latency_.add_header();
        header->CopyFrom(Transform(traffic_light->header()));
        AERROR << "Received traffic light data: run traffic light callback."
               << traffic_light->header().DebugString();
      });
  relative_map_reader_ = node_->CreateReader<relative_map::MapMsg>(
      "/apollo/relative_map",
      [this](const std::shared_ptr<relative_map::MapMsg>& map_message) {
        Header* header = relative_map_latency_.add_header();
        header->CopyFrom(Transform(map_message->header()));
        AERROR << "Received relative map data: run relative map callback."
               << map_message->header().DebugString();
      });

  pad_msg_reader_ = node_->CreateReader<planning::PadMessage>(
      "/apollo/planning/pad",
      [this](const std::shared_ptr<planning::PadMessage>& pad_msg) {
        Header* header = pad_latency_.add_header();
        header->CopyFrom(Transform(pad_msg->header()));
        AERROR << "Received pad data: run pad callback."
               << pad_msg->header().DebugString();
      });
  planning_msg_reader_ = node_->CreateReader<planning::ADCTrajectory>(
      "/apollo/planning",
      [this](const std::shared_ptr<planning::ADCTrajectory>& planning_msg) {
        Header* header = planning_latency_.add_header();
        header->CopyFrom(Transform(planning_msg->header()));
        AERROR << "Received planning data: run planning callback."
               << planning_msg->header().DebugString();
      });
  prediction_msg_reader_ = node_->CreateReader<prediction::PredictionObstacles>(
      "/apollo/prediction",
      [this](const std::shared_ptr<prediction::PredictionObstacles>&
                 prediction_msg) {
        Header* header = prediction_latency_.add_header();
        header->CopyFrom(Transform(prediction_msg->header()));
        AERROR << "Received prediction data: run prediction callback."
               << prediction_msg->header().DebugString();
      });
  perception_msg_reader_ = node_->CreateReader<perception::PerceptionObstacles>(
      "/apollo/perception/obstacles",
      [this](const std::shared_ptr<perception::PerceptionObstacles>&
                 perception_obstacle_msg) {
        Header* header = perception_latency_.add_header();
        header->CopyFrom(Transform(perception_obstacle_msg->header()));
        AERROR << "Received perception obstacle data: run perception callback."
               << perception_obstacle_msg->header().DebugString();
      });
  location_msg_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          "/apollo/localization/pose",
          [this](const std::shared_ptr<localization::LocalizationEstimate>&
                     location_msg) {
            Header* header = location_latency_.add_header();
            header->CopyFrom(Transform(location_msg->header()));
            AERROR << "Received location obstacle data: run location callback."
                   << location_msg->header().DebugString();
          });
  canbus_msg_reader_ = node_->CreateReader<canbus::Chassis>(
      "/apollo/canbus/chassis",
      [this](const std::shared_ptr<canbus::Chassis>& chassis_msg) {
        Header* header = canbus_latency_.add_header();
        header->CopyFrom(Transform(chassis_msg->header()));
        AERROR << "Received location chasis data: run chasis callback."
               << chassis_msg->header().DebugString();
      });

  exit_timer_.reset(new cyber::Timer(
      65530, [this]() { this->Process(); }, true));
  exit_timer_->Start();

  AERROR << "latency init ready!";
  return true;
}
Header LatencyComponent::Transform(const apollo::common::Header& header) {
  Header latency;
  latency.set_timestamp_sec(header.timestamp_sec());
  latency.set_module_name(header.module_name());
  latency.set_sequence_num(header.sequence_num());
  latency.set_frame_id(header.frame_id());
  return latency;
}
bool LatencyComponent::Process() {
  AERROR << "/apollo/latency.pb.txt finish";
  cyber::common::SetProtoToASCIIFile(routing_latency_.,
                                     "/apollo/routing_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(traffic_light_latency_,
                                     "/apollo/traffic_light_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(relative_map_latency_,
                                     "/apollo/relative_map_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(pad_latency_,
                                     "/apollo/pad_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(planning_latency_,
                                     "/apollo/planning_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(prediction_latency_,
                                     "/apollo/prediction_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(perception_latency_,
                                     "/apollo/perception_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(location_latency_,
                                     "/apollo/location_latency.pb.txt");
  cyber::common::SetProtoToASCIIFile(canbus_latency_,
                                     "/apollo/canbus_latency.pb.txt");

  return true;
}
}  // namespace latency
}  // namespace apollo