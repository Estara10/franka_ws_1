// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

namespace franka_gripper {

enum class GripperCompletionPolicy { kPositionTarget, kBilateralContact };

/// Plain GripperCommand is a position request regardless of the direction in
/// which the feedback currently lies. Contact semantics belong to Grasp.
constexpr GripperCompletionPolicy gripperCommandCompletionPolicy(
    double /*target_gap_m*/, double /*measured_gap_m*/) {
  return GripperCompletionPolicy::kPositionTarget;
}

constexpr GripperCompletionPolicy explicitGraspCompletionPolicy() {
  return GripperCompletionPolicy::kBilateralContact;
}

}  // namespace franka_gripper
