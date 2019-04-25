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

#include <geometry_msgs/PointStamped.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros_utilities/topic_listener.h>
#include <std_msgs/Int32.h>

TEST(TestTopicListener, BasicTest) {
  const std::string test_topic = "/TestTopicListener/basic_test";
  ros::NodeHandle node_handle;

  ros::Publisher test_pub =
      node_handle.advertise<std_msgs::Int32>(test_topic, 10);

  bookbot::TopicListener<std_msgs::Int32> topic_listener(test_topic,
                                                         node_handle);

  ros::Rate wait_time(10);
  while (test_pub.getNumSubscribers() < 1) {
    wait_time.sleep();
  }

  std_msgs::Int32 msg;
  msg.data = 1;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.data = 2;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.data = 3;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.data = 4;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  ros::Time test_time = ros::Time::now();
  msg.data = 5;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.data = 6;
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();

  std_msgs::Int32 latest_value;
  EXPECT_TRUE(topic_listener.GetLatest(&latest_value));
  EXPECT_EQ(latest_value.data, 6);

  std_msgs::Int32 closest_value;
  EXPECT_TRUE(topic_listener.GetClosest(test_time, &closest_value));
  EXPECT_EQ(closest_value.data, 5);

  std_msgs::Int32 closest_before_value;
  EXPECT_TRUE(
      topic_listener.GetClosestBefore(test_time, &closest_before_value));
  EXPECT_EQ(closest_before_value.data, 4);

  std_msgs::Int32 closest_after_value;
  EXPECT_TRUE(topic_listener.GetClosestAfter(test_time, &closest_after_value));
  EXPECT_EQ(closest_after_value.data, 5);
}

TEST(TestTopicListener, HeaderTest) {
  const std::string test_topic = "/TestTopicListener/header_test";
  ros::NodeHandle node_handle;

  ros::Publisher test_pub =
      node_handle.advertise<geometry_msgs::PointStamped>(test_topic, 10);

  auto point_interpolator =
      [](ros::Time query_time, ros::Time time_before,
         const geometry_msgs::PointStamped& message_before,
         ros::Time time_after, const geometry_msgs::PointStamped& message_after)
      -> geometry_msgs::PointStamped {
    if (time_before == time_after) {
      return message_before;
    }
    double dt_before = ros::Duration(query_time - time_before).toSec();
    double dt_after = ros::Duration(time_after - query_time).toSec();
    double interp_fraction = dt_before / (dt_before + dt_after);
    geometry_msgs::PointStamped interpolated_point;
    interpolated_point.header.frame_id = message_before.header.frame_id;
    interpolated_point.header.stamp = query_time;
    interpolated_point.point.x =
        message_before.point.x * (1 - interp_fraction) +
        message_after.point.x * interp_fraction;
    interpolated_point.point.y =
        message_before.point.y * (1 - interp_fraction) +
        message_after.point.y * interp_fraction;
    interpolated_point.point.z =
        message_before.point.z * (1 - interp_fraction) +
        message_after.point.z * interp_fraction;
    return interpolated_point;
  };

  bookbot::TopicListener<geometry_msgs::PointStamped,
                         decltype(point_interpolator)>
      topic_listener(test_topic, node_handle, 10, point_interpolator);

  ros::Rate wait_time(10);
  while (test_pub.getNumSubscribers() < 1) {
    wait_time.sleep();
  }

  // Publish in reverse order of header timestamp order
  geometry_msgs::PointStamped msg;
  msg.point.x = 6;
  msg.header.stamp = ros::Time(6);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.point.x = 5;
  msg.header.stamp = ros::Time(5);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.point.x = 4;
  msg.header.stamp = ros::Time(4);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.point.x = 3;
  msg.header.stamp = ros::Time(3);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.point.x = 2;
  msg.header.stamp = ros::Time(2);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();
  msg.point.x = 1;
  msg.header.stamp = ros::Time(1);
  test_pub.publish(msg);
  wait_time.sleep();
  ros::spinOnce();

  ros::Time test_time = ros::Time(4.7);

  geometry_msgs::PointStamped latest_value;
  EXPECT_TRUE(topic_listener.GetLatest(&latest_value));
  EXPECT_EQ(latest_value.point.x, 6);

  geometry_msgs::PointStamped closest_value;
  EXPECT_TRUE(topic_listener.GetClosest(test_time, &closest_value));
  EXPECT_EQ(closest_value.point.x, 5);

  geometry_msgs::PointStamped closest_before_value;
  EXPECT_TRUE(
      topic_listener.GetClosestBefore(test_time, &closest_before_value));
  EXPECT_EQ(closest_before_value.point.x, 4);

  geometry_msgs::PointStamped closest_after_value;
  EXPECT_TRUE(topic_listener.GetClosestAfter(test_time, &closest_after_value));
  EXPECT_EQ(closest_after_value.point.x, 5);

  geometry_msgs::PointStamped interpolated_value;
  EXPECT_TRUE(
      topic_listener.GetInterpolatedValue(test_time, &interpolated_value));
  EXPECT_EQ(interpolated_value.point.x, 4.7);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "topic_listener_test");
  return RUN_ALL_TESTS();
}
