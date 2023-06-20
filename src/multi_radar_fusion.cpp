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

#include "multi_radar_fusion/multi_radar_fusion.hpp"

#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <memory>
#include <string>
#include <vector>

using namespace std::literals;
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::placeholders::_1;

namespace
{
template <class T>
bool update_param(
  const std::vector<rclcpp::Parameter> & params, const std::string & name, T & value)
{
  const auto itr = std::find_if(
    params.cbegin(), params.cend(),
    [&name](const rclcpp::Parameter & p) { return p.get_name() == name; });

  // Not found
  if (itr == params.cend()) {
    return false;
  }

  value = itr->template get_value<T>();
  return true;
}
}  // namespace

namespace multi_radar_fusion
{
using radar_msgs::msg::RadarTracks;

MultiRadarFusionNode::MultiRadarFusionNode() : Node("multi_radar_fusion")
{
  // Parameter Server
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&MultiRadarFusionNode::onSetParam, this, _1));

  // Node Parameter
  node_param_.update_rate_hz = declare_parameter<double>("update_rate_hz", 20.0);
  node_param_.target_frame = declare_parameter<std::string>("target_frame", "base_link");
  node_param_.merge_dist_th = declare_parameter<double>("merge_dist_th", 0.1);

  // Subscriber
  sub_radar_1_ = create_subscription<RadarTracks>(
    "~/input/radar_objects_1", rclcpp::QoS{1},
    std::bind(&MultiRadarFusionNode::onRadarTracks1, this, _1));
  sub_radar_2_ = create_subscription<RadarTracks>(
    "~/input/radar_objects_2", rclcpp::QoS{1},
    std::bind(&MultiRadarFusionNode::onRadarTracks2, this, _1));
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  // Publisher
  pub_radar_ = create_publisher<RadarTracks>("~/output/radar_objects", 1);

  // Timer
  const auto update_period_ns = rclcpp::Rate(node_param_.update_rate_hz).period();
  timer_ = rclcpp::create_timer(
    this, get_clock(), update_period_ns, std::bind(&MultiRadarFusionNode::onTimer, this));
}

MultiRadarFusionNode::~MultiRadarFusionNode()
{

}

rcl_interfaces::msg::SetParametersResult MultiRadarFusionNode::onSetParam(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;

  try {
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(params, "node_params.update_rate_hz", p.update_rate_hz);
      update_param(params, "target_frame", p.target_frame);
      update_param(params, "merge_dist_th", p.merge_dist_th);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }

  result.successful = true;
  result.reason = "success";
  return result;
}

bool MultiRadarFusionNode::isDataReady()
{
  if (!radar_data_1_&&!radar_data_2_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for radar msg...");
    return false;
  }

  return true;
}

void MultiRadarFusionNode::concatTracks(RadarTracks & concated_tracks, const std::string & target_frame)
{
  RadarTracks transformed_radar_tracks_1;
  RadarTracks transformed_radar_tracks_2;

  if (radar_data_1_)
  {
    for (const auto & track : radar_data_1_ ->tracks)
    {
      RadarTrack transformed_radar_track;
      if(target_frame==radar_data_1_->header.frame_id)
      {
        transformed_radar_tracks_1.tracks.push_back(track);
        // concated_tracks.tracks.push_back(track);
      }
      else{
        transform_ = transform_listener_->getTransform(
        target_frame, radar_data_1_->header.frame_id, radar_data_1_->header.stamp,
          rclcpp::Duration::from_seconds(0.2));
        transformed_radar_track = transformRadarTrack(*transform_, track);
        transformed_radar_tracks_1.tracks.push_back(transformed_radar_track);
        // concated_tracks.tracks.push_back(transformed_radar_track);
      }
        }
  }
  if (radar_data_2_)
  {
    for (const auto & track : radar_data_2_ ->tracks)
    {
      RadarTrack transformed_radar_track;
      if(target_frame==radar_data_2_->header.frame_id)
      {
        transformed_radar_tracks_2.tracks.push_back(track);
        // concated_tracks.tracks.push_back(track);
      }else{
        transform_ = transform_listener_->getTransform(
        target_frame, radar_data_2_->header.frame_id, radar_data_2_->header.stamp,
          rclcpp::Duration::from_seconds(0.2)); 
        transformed_radar_track = transformRadarTrack(*transform_, track);
        transformed_radar_tracks_2.tracks.push_back(transformed_radar_track);
        // concated_tracks.tracks.push_back(transformed_radar_track);
      }
    }
  }
  // do remove duplicated tracks
  concated_tracks = removeDuplicateTracks(transformed_radar_tracks_1, transformed_radar_tracks_2, node_param_.merge_dist_th);
  concated_tracks.header = radar_data_1_->header;
  concated_tracks.header.frame_id = target_frame;
}

RadarTracks MultiRadarFusionNode::removeDuplicateTracks(const RadarTracks & radar_tracks_1, const RadarTracks & radar_tracks_2, const double & merge_distance_threshold)
{
  RadarTracks filtered_tracks;
  filtered_tracks.header = radar_tracks_1.header;
  filtered_tracks.tracks = radar_tracks_1.tracks;

  for (const auto & track_2 : radar_tracks_2.tracks) {
    bool is_duplicated = false;
    for (const auto & track_1 : radar_tracks_1.tracks) {
      double distance = calculateEuclideanDistance(track_1.position, track_2.position);
      if (distance < merge_distance_threshold) 
      {
        is_duplicated = true;
        break;
      }
    }
    if (!is_duplicated) {
      filtered_tracks.tracks.push_back(track_2);
    }
  }
  return filtered_tracks;
}

double MultiRadarFusionNode::calculateEuclideanDistance(
  const geometry_msgs::msg::Point & point1, const geometry_msgs::msg::Point & point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double dz = point1.z - point2.z;

  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void MultiRadarFusionNode::onRadarTracks1(const RadarTracks::ConstSharedPtr & msg)
{
  radar_data_1_ = std::make_shared<RadarTracks>(*msg);
}

void MultiRadarFusionNode::onRadarTracks2(const RadarTracks::ConstSharedPtr & msg)
{
  radar_data_2_ = std::make_shared<RadarTracks>(*msg);
}

void MultiRadarFusionNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  RadarTracks radar_tracks;
  concatTracks(radar_tracks, node_param_.target_frame);

  if (!radar_tracks.tracks.empty()) {
    pub_radar_->publish(radar_tracks);
  }
}

RadarTrack MultiRadarFusionNode::transformRadarTrack(const geometry_msgs::msg::TransformStamped & tf, const RadarTrack & radar_track)
{
  RadarTrack transformed_radar_track;
  // unchanged variables
  transformed_radar_track.uuid = radar_track.uuid;
  transformed_radar_track.size = radar_track.size;
  transformed_radar_track.classification = radar_track.classification;
  transformed_radar_track.position_covariance = radar_track.position_covariance;
  transformed_radar_track.velocity_covariance = radar_track.velocity_covariance;
  transformed_radar_track.acceleration_covariance = radar_track.acceleration_covariance;
  // changed variables
  geometry_msgs::msg::PoseStamped pose_in;
  geometry_msgs::msg::PoseStamped pose_out;
  pose_in.pose.position = radar_track.position;

  geometry_msgs::msg::Vector3Stamped vel_in;
  geometry_msgs::msg::Vector3Stamped vel_out;
  vel_in.vector = radar_track.velocity;

  geometry_msgs::msg::Vector3Stamped accel_in;
  geometry_msgs::msg::Vector3Stamped accel_out;
  accel_in.vector = radar_track.acceleration;

  tf2::doTransform(pose_in, pose_out, tf);
  tf2::doTransform(vel_in, vel_out, tf);
  tf2::doTransform(accel_in, accel_out, tf);
  
  transformed_radar_track.position = pose_out.pose.position;
  transformed_radar_track.velocity = vel_out.vector;
  transformed_radar_track.acceleration = accel_out.vector;

  return transformed_radar_track;
}

}  // namespace multi_radar_fusion
