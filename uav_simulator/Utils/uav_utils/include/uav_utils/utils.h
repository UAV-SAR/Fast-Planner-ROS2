#ifndef __UAV_UTILS_H
#define __UAV_UTILS_H

#include <rclcpp/rclcpp.hpp>
#include <cassert>
#include <uav_utils/converters.h>
#include <uav_utils/geometry_utils.h>

namespace uav_utils
{

// Checks if value is in [low, high]
template <typename T, typename T2>
bool in_range(T value, const T2& low, const T2& high)
{
  assert(low < high && "in_range: low must be less than high");
  return (low <= value) && (value <= high);
}

// Checks if value is in [-limit, limit]
template <typename T, typename T2>
bool in_range(T value, const T2& limit)
{
  assert(limit > 0 && "in_range: limit must be positive");
  return in_range(value, -limit, limit);
}

// Limits value to [low, high]
template <typename T, typename T2>
void limit_range(T& value, const T2& low, const T2& high)
{
  assert(low < high && "limit_range: low must be less than high");
  if (value < low)
    value = low;
  if (value > high)
    value = high;
}

// Limits value to [-limit, limit]
template <typename T, typename T2>
void limit_range(T& value, const T2& limit)
{
  assert(limit > 0 && "limit_range: limit must be positive");
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace uav_utils

#endif
