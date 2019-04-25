// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BOOKBOT_ROS_UTILITIES_PARAMETER_LOADING_H_
#define BOOKBOT_ROS_UTILITIES_PARAMETER_LOADING_H_

#include <ros/ros.h>

namespace bookbot {

/**
 * @brief  Base case for ReadParameters parameter pack expansion. See below for
 * usage and full description.
 */
template <typename... NamedParameterTypes>
bool ReadParameters(const ros::NodeHandle& node_handle,
                    NamedParameterTypes... remaining_parameters) {
  // Base case only gets called when ParameterTypes is an empty parameter pack
  if (sizeof...(remaining_parameters) > 0) {
    ROS_ERROR_STREAM("ReadParameters called with invalid structure");
    return false;
  }
  return true;
}

/**
 * @brief ReadParameters is a utility meant to provide a slightly more sane
 * way to load a large set of parameters from the ros parameter server.
 *
 * Usage:
 *
 * bool success = ReadParameters(node_handle,
 *                               first_param_name, &first_param,
 *                               second_param_name, &second_param,
 *                               third_param_name, &third_param,
 *                                            ...
 *                               last_param_name, &last_param);
 *
 * @tparam StringType            Type used for parameter names e.g. std::string
 * @tparam ParameterType         Type of the currently expanded parameter
 * @tparam NamedParameterTypes   Types of remaining names/parameters
 * @param node_handle            Ros nodehandle used to access parameter server
 * @param name                   Name for currently expanded parameter
 * @param parameter              Currently expanded parameter
 * @param remaining_parameters   Parameter pack of remaining names/parameters
 * @return true                  All parameters were loaded successfuly
 * @return false                 At least one parameter failed to load or there
 *                               was a structural error in how the function was
 *                               called
 */
template <typename StringType, typename ParameterType,
          typename... NamedParameterTypes>
bool ReadParameters(const ros::NodeHandle& node_handle, StringType name,
                    ParameterType* parameter,
                    NamedParameterTypes... remaining_parameters) {
  if (!node_handle.getParam(name, *parameter)) {
    ROS_ERROR_STREAM("Failed to read parameter:" << name);
    return false;
  }
  ROS_INFO_STREAM("Loaded parameter:" << name << " with value:" << *parameter);
  return ReadParameters(node_handle, remaining_parameters...);
}

}  // namespace bookbot

#endif  // BOOKBOT_ROS_UTILITIES_PARAMETER_LOADING_H_
