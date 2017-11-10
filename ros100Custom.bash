#!/bin/bash
SESSION=$USER

if [ "$#" -ne 2 ] 
then
  echo "Usage : ./ros100Custom.bash [prbs] [saturation]"
  exit 0
else
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
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs $1 100 $2" C-m
fi


# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
