// Copyright 2023 Pixmoving, Inc. 
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

#ifndef __MULTI_RADAR_FUSION__HPP__
#define __MULTI_RADAR_FUSION__HPP__

#include "rclcpp/rclcpp.hpp"

#include "tier4_autoware_utils/tier4_autoware_utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "autoware_auto_perception_msgs/msg/object_classification.hpp"
#include "autoware_auto_perception_msgs/msg/shape.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_object_kinematics.hpp"
#include "autoware_auto_perception_msgs/msg/tracked_objects.hpp"
#include "radar_msgs/msg/radar_tracks.hpp"
#include "radar_msgs/msg/radar_track.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

namespace multi_radar_fusion
{
using autoware_auto_perception_msgs::msg::ObjectClassification;
using autoware_auto_perception_msgs::msg::Shape;
using autoware_auto_perception_msgs::msg::TrackedObject;
using autoware_auto_perception_msgs::msg::TrackedObjectKinematics;
using autoware_auto_perception_msgs::msg::TrackedObjects;
using radar_msgs::msg::RadarTrack;
using radar_msgs::msg::RadarTracks;

class MultiRadarFusionNode : public rclcpp::Node
{
public: 
  explicit MultiRadarFusionNode();
  ~MultiRadarFusionNode();
  struct NodeParam
  {
    double update_rate_hz{};
    std::string target_frame;
    double merge_dist_th;
  };

private:
  // Subscriber
  rclcpp::Subscription<RadarTracks>::ConstSharedPtr sub_radar_1_;
  rclcpp::Subscription<RadarTracks>::ConstSharedPtr sub_radar_2_;

  // using SyncPolicy =
  //   message_filters::sync_policies::ApproximateTime<RadarTracks, RadarTracks>;
  // using Sync = message_filters::Synchronizer<SyncPolicy>;
  // typename std::shared_ptr<Sync> sync_ptr_;

  // Callback
  // void onRadarTracks(const RadarTracks::ConstSharedPtr radar_1, const RadarTracks::ConstSharedPtr radar_2);
  void onRadarTracks1(const RadarTracks::ConstSharedPtr & radar_msg);
  void onRadarTracks2(const RadarTracks::ConstSharedPtr & radar_msg);

  void concatTracks(RadarTracks & concated_tracks, const std::string & target_frame);

  RadarTracks removeDuplicateTracks(const RadarTracks & radar_tracks_1, const RadarTracks & radar_tracks_2, const double & merge_distance_threshold);
  double calculateEuclideanDistance(
    const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2);

  // Data  Buffer
  std::shared_ptr<RadarTracks> radar_data_1_;
  std::shared_ptr<RadarTracks> radar_data_2_;

  // tf
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_;

  // Publishser
  rclcpp::Publisher<RadarTracks>::SharedPtr pub_radar_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_{};

  bool isDataReady();
  void onTimer();

  // Parameter Server
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onSetParam(
    const std::vector<rclcpp::Parameter> & params);

  // Parameter
  NodeParam node_param_{};

  RadarTrack transformRadarTrack(
    const geometry_msgs::msg::TransformStamped & tf, const RadarTrack & radar_track);
};
}

#endif
