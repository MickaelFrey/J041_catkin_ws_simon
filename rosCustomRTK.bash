#!/bin/bash
SESSION=$USER

#no args : default = 0prbs 2000sat 100hz no log
if [ "$#" -eq 0 ] 
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
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs" C-m

#1 arg : prbs, default = 2000sat 100hz no log
elif [ "$#" -eq 1 ] 
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
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs $1" C-m

#2 args : prbs, saturation, default = 100hz no log
elif [ "$#" -eq 2 ] 
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
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs $1 $2" C-m

#3 args : prbs, saturation, frequency, default = no log
elif [ "$#" -eq 3 ]
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
  tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs $1 $2 $3" C-m

#4 args : prbs, saturation, frequency, logging = on
elif [ "$#" -eq 4 ]
then
  if [ "$4" == "-log" ]
  then
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -v
    tmux select-pane -t 0
    tmux split-window -h
    #ROSCORE
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    #ROSBAG
    tmux select-pane -t 1
    tmux send-keys "sleep 10" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "cd /home/pi/bagfiles" C-m
    tmux send-keys "rosbag record -a" C-m
    #IMU
    tmux select-pane -t 3
    tmux send-keys "sleep 5" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_imu imu_pub $3" C-m
    #REMOTE
    tmux select-pane -t 2
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_pub_sub_sat_prbs $1 $2 $3" C-m
    #GPS
 	tmux select-pane -t 4
    tmux send-keys "sudo ifconfig usb0 192.168.2.2" C-m
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun gps_rtk gps_rtk" C-m
  else
    echo "Usage : ./rosCustomRTK.bash [prbs] [saturation] [freq] [-log]"
    exit 0
  fi
else
    echo "Usage : ./rosCustomRTK.bash [prbs] [saturation] [freq] [-log]"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION
