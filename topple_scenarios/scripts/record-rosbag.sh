#!/bin/sh

# This script records all PR2 base dynamics and marker topics

rosbag record \
  -o "`rospack find topple_scenarios`/episodes" \
   /topple_episode_monitor/topple_episode_part
