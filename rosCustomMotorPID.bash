#!/bin/bash
SESSION=$USER

#no log
if [ "$#" -eq 6 ] 
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
    tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_motor_PID $1 $2 $3 $4 $5 $6" C-m

#logging = on
elif [ "$#" -eq 7 ]
then
  if [ "$7" == "-log" ]
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
    tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_motor_PID $1 $2 $3 $4 $5 $6" C-m
    tmux split-window -v
    tmux send-keys "sleep 10" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "cd /home/pi/bagfiles" C-m
    tmux send-keys "rosbag record -a" C-m

  else
    echo "Usage : ./rosCustom.bash [freq] [saturation] [Kp] [Ki] [Kd] [PRBS] [-log]"
    exit 0
  fi
else
    echo "Usage : ./rosCustom.bash [freq] [saturation] [Kp] [Ki] [Kd] [PRBS] [-log]"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
