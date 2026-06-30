/*
 * Author
 * Copyright (c) 2018, Simbe Robotics, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Steven Macenski */

#ifndef SLAM_TOOLBOX__SERIALIZATION_HPP_
#define SLAM_TOOLBOX__SERIALIZATION_HPP_

#include <sys/stat.h>
#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "karto_sdk/Mapper.h"

namespace serialization
{

inline bool fileExists(const std::string & name)
{
  struct stat buffer;
  return stat(name.c_str(), &buffer) == 0;
}

inline bool write(
  const std::string & filename,
  karto::Mapper & mapper,
  karto::Dataset & dataset,
  rclcpp::Node::SharedPtr node)
{
  // [symros] serialization ablated: boost binary archives removed to avoid the
  // libstdc++/libc++ ABI mismatch. Pose-graph save is a no-op here.
  (void)filename;
  (void)mapper;
  (void)dataset;
  RCLCPP_WARN(node->get_logger(),
    "[symros] serialization::write ablated (no-op); pose-graph not saved.");
  return false;
}

inline bool read(
  const std::string & filename,
  karto::Mapper & mapper,
  karto::Dataset & dataset,
  rclcpp::Node::SharedPtr node)
{
  // [symros] serialization ablated: see write(). Returning false makes callers
  // (deserialize service, merge_maps) fail gracefully instead of using empty state.
  (void)filename;
  (void)mapper;
  (void)dataset;
  RCLCPP_WARN(node->get_logger(),
    "[symros] serialization::read ablated (no-op); no pose-graph loaded.");
  return false;
}

}  // namespace serialization

#endif  // SLAM_TOOLBOX__SERIALIZATION_HPP_
