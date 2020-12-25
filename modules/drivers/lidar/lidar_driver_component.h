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

#include "modules/drivers/lidar/proto/config.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/lidar/common/driver_factory/lidar_driver_factory.h"

namespace apollo {
namespace drivers {
namespace lidar {

class LidarDriverComponent : public ::apollo::cyber::Component<> {
 public:
  LidarDriverComponent();
  ~LidarDriverComponent(){};
  bool Init() override;

 private:
  std::shared_ptr<LidarDriverFactory> lidar_factory_;
  apollo::drivers::lidar::config conf_;
  std::shared_ptr<::apollo::cyber::Node> node_;
  std::unique_ptr<LidarDriver> driver_;
};

CYBER_REGISTER_COMPONENT(LidarDriverComponent)

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
