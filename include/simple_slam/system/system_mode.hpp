#ifndef SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_
#define SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_

#include <string>

namespace simple_slam
{

// 系统模式先统一抽象出来，后面接入纯定位和离线建图时不用再改节点外形。
enum class SystemMode
{
  kMapping,
  kLocalization,
};

inline SystemMode ParseSystemMode(const std::string & mode)
{
  if (mode == "localization") {
    return SystemMode::kLocalization;
  }
  return SystemMode::kMapping;
}

inline const char * ToString(const SystemMode mode)
{
  return mode == SystemMode::kLocalization ? "localization" : "mapping";
}

}  // namespace simple_slam

#endif  // SIMPLE_SLAM__SYSTEM__SYSTEM_MODE_HPP_
