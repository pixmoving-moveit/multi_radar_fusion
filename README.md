# Multi Radar Fusion

## Overview

This pachage contains a sensor fusion module for radar detected objects. The fusion node can:
- **Concatenate the tracks of two radars into one track.**
- **Coordinate the radar tracks and calculate the Euclidean distance between any two points of radar 1 and radar 2.**
- **Use the Euclidean distance to determine whether the track is repeated and remove the duplicate track.**

## Design

### Parameters

| Name                     | Type   | Description                                                                                                                                                                                                                                                                      | Default value |
| :----------------------- | :----- | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :------------ |
|  update_rate_hz  | double  |  The update rate [hz].  |  20.0  |
|  target_frame  |  string  |  The header frame of output topic.  |  base_link  |
|  merge_dist_th  | double  |  Distance threshold used to determine whether to remove duplicate data.  |  0.1  |

### Input

| Name                    | Type                                                 | Description                                                            |
| ----------------------- | ---------------------------------------------------- | ---------------------------------------------------------------------- |
|  `~/input/radar_objects_1`  |  radar_msgs/msg/RadarTracks.msg  |  Input radar topic 1.  |
|  `~/input/radar_objects_2`  |  radar_msgs/msg/RadarTracks.msg  |  Input radar topic 2.  |

### Output

| Name                    | Type                                                 | Description                                                            |
| ----------------------- | ---------------------------------------------------- | ---------------------------------------------------------------------- |
|  `~/output/radar_objects`  |  radar_msgs/msg/RadarTracks.msg  |  Output radar topic.  |
