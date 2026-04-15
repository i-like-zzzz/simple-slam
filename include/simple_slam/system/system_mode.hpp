// Copyright 2026 zwc
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

#ifndef SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_
#define SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_

#include <string>

namespace simple_slam {

// 系统模式先统一抽象出来，后面接入纯定位和离线建图时不用再改节点外形。
enum class SystemMode {
  kMapping,
  kLocalization,
};

inline SystemMode ParseSystemMode(const std::string& mode) {
  if (mode == "localization") {
    return SystemMode::kLocalization;
  }
  return SystemMode::kMapping;
}

inline const char* ToString(const SystemMode mode) {
  return mode == SystemMode::kLocalization ? "localization" : "mapping";
}

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_
