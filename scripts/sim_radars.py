#! /usr/bin/python3

import random
import numpy as np
import rclpy
from radar_msgs.msg import RadarTracks
from radar_msgs.msg import RadarTrack

from rclpy.node import Node

import uuid

class SimRadarsPublisher(Node):
  def __init__(self):
    super().__init__('sim_radars_publish')
    self.radar_1_publisher_ = self.create_publisher(RadarTracks, '/front_radar/tracks', 10)
    self.radar_2_publisher_ = self.create_publisher(RadarTracks, '/rear_radar/tracks', 10)
    timer_period = 0.05  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def construct_random_radar_track(self, i, x, y, factor):
    radar_track = RadarTrack()
    uuid_list = list()
    for j in range(16):
      uuid_list.append(2*i+3*j+factor)
    radar_track.uuid.uuid = np.array(uuid_list).astype('u1')
    radar_track.position.x = float(x)
    radar_track.position.y = float(y)
    radar_track.position.z = 0.0
    radar_track.size.x = 1.0
    radar_track.size.y = 1.0
    radar_track.size.z = 1.0
    return radar_track

  def timer_callback(self):
    radar_tracks_1 = RadarTracks()
    radar_tracks_2 = RadarTracks()
    for i in range(10):
      radar_tracks_1.tracks.append(self.construct_random_radar_track(i, i, 2*i, 1))
    for i in range(10):
      radar_tracks_2.tracks.append(self.construct_random_radar_track(i, i, 2*i, 2)) 
    radar_tracks_1.header.stamp = self.get_clock().now().to_msg()
    radar_tracks_1.header.frame_id = "front_ars408"
    self.radar_1_publisher_.publish(radar_tracks_1)
    radar_tracks_2.header = radar_tracks_1.header
    radar_tracks_2.header.frame_id = "rear_ars408"
    self.radar_2_publisher_.publish(radar_tracks_2)


def main(args=None):
  rclpy.init(args=args)

  sim_radar_publisher = SimRadarsPublisher()

  rclpy.spin(sim_radar_publisher)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  sim_radar_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()