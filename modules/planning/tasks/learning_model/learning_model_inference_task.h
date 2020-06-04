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

/**
 * @file
 **/

#pragma once

#include <utility>
#include <vector>

#include "modules/planning/common/trajectory_evaluator.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

class LearningModelInferenceTask : public Task {
 public:
  explicit LearningModelInferenceTask(const TaskConfig &config);

  apollo::common::Status Execute(
      Frame *frame, ReferenceLineInfo *reference_line_info) override;

 private:
  apollo::common::Status Process(Frame *frame);

  void ConvertADCFutureTrajectory(
      const std::vector<TrajectoryPointFeature>& trajectory,
      std::vector<common::TrajectoryPoint>* adc_future_trajectory);

 private:
  TrajectoryEvaluator trajectory_evaluator_;
};

}  // namespace planning
}  // namespace apollo
