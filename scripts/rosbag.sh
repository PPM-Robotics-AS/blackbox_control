#!/bin/sh

rosbag play $1 -s $2 --skip-empty=60 __name:=blackbox_player
