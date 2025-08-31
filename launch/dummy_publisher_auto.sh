#!/bin/bash
# filepath: /home/oga/ros2_ksenos/src/ksenos_ground/launch/dummy_publisher.sh
ros2 topic pub /sbus_data ksenos_ground_msgs/msg/SbusData "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, aileron_r: 0.0, elevator: 0.0, throttle: 0.0, rudder: 0.0, aileron_l: 0.0, dropping_device: 0, autopilot_mode: 'horizontal rotation', is_lost_frame: false, is_failsafe: false}" &
ros2 topic pub /controller/lat/calc/turning_radius std_msgs/msg/Float32 "{data: 6.0}" &
wait