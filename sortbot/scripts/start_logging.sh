#!/bin/bash
if [ $# -ne 1 ] ; then
  echo "hai plz give me the name of ur dataset like this:"
  echo "  start_logging.sh DATASET_NAME"
  echo "k bai"
  exit 1
fi
cd `rospack find sortbot`
scripts/aim_head.py
scripts/projector_on.sh
scripts/start_laser.sh
export DATASET_NAME=$1
roslaunch `rospack find sortbot`/launch/collect_data.launch
