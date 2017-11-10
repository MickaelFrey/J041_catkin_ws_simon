#!/bin/bash
SESSION=$USER

#2 args : frequency, saturation, default = no log
if [ "$#" -eq 5 ] 
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
    tmux send-keys "rosrun navio2_imu imu_pub $(($1-1))" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_PID2 $1 $2 $3 $4 $5" C-m

#3 args : frequency, saturation, logging = on
elif [ "$#" -eq 6 ]
then
  if [ "$6" == "-log" ]
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
    tmux send-keys "rosrun navio2_imu imu_pub $(($1-1))" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_PID2 $1 $2 $3 $4 $5" C-m
    tmux split-window -v
    tmux send-keys "sleep 10" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "cd /home/pi/bagfiles" C-m
    tmux send-keys "rosbag record -a" C-m

  else
    echo "Usage : ./rosCustom.bash [freq] [saturation] [Kp] [Ki] [Kd] [-log]"
    exit 0
  fi
else
    echo "Usage : ./rosCustom.bash [freq] [saturation] [Kp] [Ki] [Kd] [-log]"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
