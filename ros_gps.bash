#!/bin/bash
SESSION=$USER

if [ "$#" -ne 1 ] 
then
  tmux -2 new-session -d -s $SESSION
  tmux new-window -t $SESSION:1 -n 'ROS'
  tmux split-window -v
  tmux select-pane -t 0
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "roscore" C-m
  tmux select-pane -t 1
  tmux send-keys "sleep 5" C-m
  tmux send-keys "sudo -i" C-m
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "rosrun navio2_remote remote_pub_sub" C-m
elif [ "$1" == "-log" ]
then
  tmux -2 new-session -d -s $SESSION
  tmux new-window -t $SESSION:1 -n 'ROS'
  tmux split-window -h
  tmux select-pane -t 0
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "roscore" C-m
  tmux select-pane -t 1
  tmux send-keys "sleep 5" C-m
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "rosrun navio2_imu imu_pub" C-m
  tmux split-window -v
  tmux send-keys "sleep 5" C-m
  tmux send-keys "rosrun navio2_gps gps_pub" C-m
  tmux select-pane -t 0
  tmux split-window -v
  tmux send-keys "sleep 5" C-m
  tmux send-keys "sudo -i" C-m
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs 120 1600" C-m
  tmux split-window -v
  tmux send-keys "sleep 10" C-m
  tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
  tmux send-keys "cd /home/pi/bagfiles" C-m
  tmux send-keys "rosbag record -a" C-m
else 
  echo "Usage : ./ros.bash [-log]"
  exit 0
fi


# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
