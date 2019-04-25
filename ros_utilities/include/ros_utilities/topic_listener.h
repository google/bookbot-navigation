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

#ifndef BOOKBOT_ROS_UTILITIES_TOPIC_LISTENER_H_
#define BOOKBOT_ROS_UTILITIES_TOPIC_LISTENER_H_
#include <ros/message_event.h>
#include <ros/ros.h>

#include <map>
#include <mutex>
#include <type_traits>

namespace bookbot {

namespace detail {

// SFINAE to check if MessageType has a header member
template <typename MessageType, typename = int>
struct HasHeader : std::false_type {};

template <typename MessageType>
struct HasHeader<MessageType, decltype((void)MessageType::header, 0)>
    : std::true_type {};

// SFINAE to retrieve header time for MessageTypes with a header and receipt
// time for MessageTypes without a header
template <typename MessageType,
          typename std::enable_if<HasHeader<MessageType>::value, int>::type = 0>
ros::Time GetTime(const ros::MessageEvent<MessageType const>& message_event) {
  // MessageType has a header member - return header time
  return message_event.getMessage()->header.stamp;
}

template <
    typename MessageType,
    typename std::enable_if<!HasHeader<MessageType>::value, int>::type = 0>
ros::Time GetTime(const ros::MessageEvent<MessageType const>& message_event) {
  // MessageType is headerless, return message_event time
  return message_event.getReceiptTime();
}

}  // namespace detail

/**
 * @brief Default implementation of a user-supplied interpolation function. This
 * default version simply asserts since there is no way to reasonably
 * interpolate arbitrary message types.
 *
 * @Note: For interpolation the user must provide a lambda or functor with this
 * same signature to perform interpolation between adjacent messages in the
 * queue.
 */
template <typename MessageType>
struct DefaultInterpolationFunctor {
  MessageType operator()(ros::Time query_time, ros::Time time_before,
                         const MessageType& message_before,
                         ros::Time time_after,
                         const MessageType& message_after) {
    assert(false && "Need to provide custom interpolation functor");
  }
};

/**
 * @brief  Object that listens on a topic and allows for user-custabizable
 * interpolation or retrieval of a value from a fixed-size queue of the latest
 * messages.
 *
 * Messages are ordered by header.stamp (if a header is available) or receipt
 * time (if no header). The GetLatest() member function can be used
 * to access the most recent message in the queue and the GetClosest* member
 * functions can be used to access messages bracketing a given query_time. The
 * GetInterpolatedValue() member function calls a user provided interpolation
 * functor with the bracketing messages for a given query time and returns the
 * interpolated result.
 *
 * @tparam MessageType       The message type.
 * @tparam RetrievalFunctor  The functor used to extract a synchronized value
 *                           from the queue of messages.
 */
template <typename MessageType, typename InterpolationFunctor =
                                    DefaultInterpolationFunctor<MessageType>>
class TopicListener {
 public:
  TopicListener(
      const std::string& topic, ros::NodeHandle node_handle,
      int queue_size = 100,
      InterpolationFunctor interpolation_functor = InterpolationFunctor())
      : queue_size_(queue_size), interpolation_functor_(interpolation_functor) {
    subscriber_ = node_handle.subscribe(
        topic, queue_size,
        &TopicListener<MessageType, InterpolationFunctor>::Callback, this);
  }

  /**
   * @brief Get the most recent message in the queue
   *
   * @return false if the queue is empty
   */
  bool GetLatest(MessageType* message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (sorted_message_queue_.empty()) {
      return false;
    }
    if (message) {
      *message = std::prev(std::end(sorted_message_queue_))->second;
    }
    return true;
  }

  /**
   * @brief Get the message with the closest time to the query time
   *
   * @return false if the queue is empty
   */
  bool GetClosest(ros::Time query_time, MessageType* message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    typename std::map<ros::Time, MessageType>::const_iterator
        message_before_iter;
    typename std::map<ros::Time, MessageType>::const_iterator
        message_after_iter;
    if (!GetBracketingValueIterators(query_time, &message_before_iter,
                                     &message_after_iter)) {
      return false;
    }
    if (!message) {
      return true;
    }
    if (ros::Duration(query_time - message_before_iter->first).toSec() <
        ros::Duration(message_after_iter->first - query_time).toSec()) {
      *message = message_before_iter->second;
    } else {
      *message = message_after_iter->second;
    }
    return true;
  }

  /**
   * @brief Get the message with the closest time before than the query time
   *
   * @return false if the queue is empty
   */
  bool GetClosestBefore(ros::Time query_time, MessageType* message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    typename std::map<ros::Time, MessageType>::const_iterator
        message_before_iter;
    typename std::map<ros::Time, MessageType>::const_iterator
        message_after_iter;
    if (!GetBracketingValueIterators(query_time, &message_before_iter,
                                     &message_after_iter)) {
      return false;
    }
    if (message) {
      *message = message_before_iter->second;
    }
    return true;
  }

  /**
   * @brief Get the message with the closest time after the query time
   *
   * @return false if the queue is empty
   */
  bool GetClosestAfter(ros::Time query_time, MessageType* message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    typename std::map<ros::Time, MessageType>::const_iterator
        message_before_iter;
    typename std::map<ros::Time, MessageType>::const_iterator
        message_after_iter;
    if (!GetBracketingValueIterators(query_time, &message_before_iter,
                                     &message_after_iter)) {
      return false;
    }
    if (message) {
      *message = message_after_iter->second;
    }
    return true;
  }

  /**
   * @brief Get the interpolated message for the query time using the
   * user-supplied InterpolationFunctor.
   * @Note This function will throw an assert if no InterpolationFunctor has
   * been provided.
   *
   * @return false if the queue is empty
   */
  bool GetInterpolatedValue(ros::Time query_time, MessageType* message) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    typename std::map<ros::Time, MessageType>::const_iterator
        message_before_iter;
    typename std::map<ros::Time, MessageType>::const_iterator
        message_after_iter;
    if (!GetBracketingValueIterators(query_time, &message_before_iter,
                                     &message_after_iter)) {
      return false;
    }
    if (message) {
      *message = interpolation_functor_(
          query_time, message_before_iter->first, message_before_iter->second,
          message_after_iter->first, message_after_iter->second);
    }
    return true;
  }

 private:
  void Callback(const ros::MessageEvent<MessageType const>& message_event) {
    ros::Time timestamp = detail::GetTime(message_event);
    std::lock_guard<std::mutex> lock(queue_mutex_);
    sorted_message_queue_.emplace_hint(std::end(sorted_message_queue_),
                                       timestamp, *message_event.getMessage());
    while (sorted_message_queue_.size() > queue_size_) {
      sorted_message_queue_.erase(std::begin(sorted_message_queue_));
    }
  }

  bool GetBracketingValueIterators(
      ros::Time query_time,
      typename std::map<ros::Time, MessageType>::const_iterator*
          message_before_iter,
      typename std::map<ros::Time, MessageType>::const_iterator*
          message_after_iter) {
    // Note: assumes queue_mutex_ is locked in the calling scope
    if (sorted_message_queue_.empty() || !message_before_iter ||
        !message_after_iter) {
      return false;
    }
    auto iter = sorted_message_queue_.lower_bound(query_time);
    if (iter == std::begin(sorted_message_queue_)) {
      *message_before_iter = iter;
      *message_after_iter = iter;
      return true;
    }
    if (iter == std::end(sorted_message_queue_)) {
      *message_before_iter = std::prev(iter);
      *message_after_iter = std::prev(iter);
      return true;
    }
    *message_before_iter = std::prev(iter);
    *message_after_iter = iter;
    return true;
  }

  mutable std::mutex queue_mutex_;
  int queue_size_;
  InterpolationFunctor interpolation_functor_;
  ros::Subscriber subscriber_;
  std::map<ros::Time, MessageType> sorted_message_queue_;
};

}  // namespace bookbot

#endif  // BOOKBOT_ROS_UTILITIES_TOPIC_LISTENER_H_
