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
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>

#include "modules/latency/proto/latency.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/proto/pad_msg.pb.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

#include "cyber/class_loader/class_loader.h"
#include "cyber/component/component.h"
#include "cyber/component/timer_component.h"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
/**
 * @namespace apollo::latency
 * @brief apollo::latency
 */
namespace apollo {
namespace latency {
using apollo::perception::TrafficLightDetection;
using apollo::routing::RoutingRequest;
class LatencyComponent : public apollo::cyber::Component<> {
 public:
  LatencyComponent() = default;
  bool Init() override;

 private:
  bool Process();
  Header Transform(const apollo::common::Header& header);
  apollo::cyber::Time init_time_;
  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>>
      traffic_light_reader_;
  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_msg_reader_;
  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;
  std::shared_ptr<cyber::Reader<planning::ADCTrajectory>> planning_msg_reader_;
  std::shared_ptr<cyber::Reader<prediction::PredictionObstacles>>
      prediction_msg_reader_;
  std::shared_ptr<cyber::Reader<perception::PerceptionObstacles>>
      perception_msg_reader_;
  std::shared_ptr<cyber::Reader<localization::LocalizationEstimate>>
      location_msg_reader_;
  std::shared_ptr<cyber::Reader<canbus::Chassis>> canbus_msg_reader_;

  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
  std::shared_ptr<cyber::Reader<apollo::planning::ADCTrajectory>>
      trajectory_reader_;
  Latency routing_latency_;
  Latency traffic_light_latency_;
  Latency relative_map_latency_;
  Latency pad_latency_;
  Latency planning_latency_;
  Latency prediction_latency_;
  Latency perception_latency_;
  Latency location_latency_;
  Latency canbus_latency_;
  std::unique_ptr<cyber::Timer> exit_timer_;
};
CYBER_REGISTER_COMPONENT(LatencyComponent);
}  // namespace latency
}  // namespace apollo