#!/bin/bash
SESSION=$USER

#no log, auto run, eric's code
if [ "$#" -eq 1 ] && [ "$1" == 'auto' ]
then
    echo "Here we go in AUTO MODE, freq= 50, MaxThrottlePwm=1600, Kp=0.7, Ki=0.2, Kd=0.2"
    sleep 2
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    tmux select-pane -t 1
    tmux send-keys "sleep 5" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_imu imu_pub 49" C-m
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_remote remote_multiCtr 50 1600 0.7 0.2 0.2" C-m
    
#no log, gps run only, no bike control
elif [ "$#" -eq 1 ] && [ "$1" == 'gps' ]
then
    echo "Here we go in gps observation"
    sleep 2
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    tmux select-pane -t 1
    tmux send-keys "sudo ifconfig usb0 192.168.2.2" C-m
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun gps_rtk gps_rtk2" C-m

#no log, eric's code and gps launch
elif [ "$#" -eq 1 ] && [ "$1" == 'all' ]
then
    echo "Here we go in auto mode and gps information: freq= 50, MaxThrottlePwm=1600, Kp=0.7, Ki=0.2, Kd=0.2"
    sleep 2
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    
    tmux select-pane -t 1
    tmux send-keys "sleep 5" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_imu imu_pub 49" C-m
    
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    #tmux send-keys "rosrun navio2_remote remote_multiCtr 50 1600 0.7 0.2 0.2" C-m
    tmux send-keys "rosrun navio2_remote remote_multiCtr_KalmanFiltering 50 1600 0.7 0.2 0.2 46.51849177 6.56666458" C-m
    
    tmux select-pane -t 1
    tmux split-window -v
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "sudo ifconfig usb0 192.168.2.2" C-m
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun gps_rtk gps_rtk2" C-m
    
#log files, eric's code and gps launch
elif [ "$#" -eq 1 ] && [ "$1" == 'alllog' ]
then
    echo "Here we go in auto mode and gps information (log recorded): freq= 50, MaxThrottlePwm=1600, Kp=0.7, Ki=0.2, Kd=0.2"
    sleep 2
    tmux -2 new-session -d -s $SESSION
    tmux new-window -t $SESSION:1 -n 'ROS'
    tmux split-window -h
    
    tmux select-pane -t 0
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "roscore" C-m
    
    tmux select-pane -t 1
    tmux send-keys "sleep 5" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun navio2_imu imu_pub 49" C-m
    
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    #tmux send-keys "rosrun navio2_remote remote_multiCtr 50 1600 0.7 0.2 0.2" C-m
    tmux send-keys "rosrun navio2_remote remote_multiCtr_KalmanFiltering 50 1600 0.7 0.2 0.2 46.51849177 6.56666458" C-m
    
    tmux select-pane -t 1
    tmux split-window -v
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "sudo ifconfig usb0 192.168.2.2" C-m
    tmux send-keys "sleep 5" C-m
    tmux send-keys "sudo -i" C-m
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "rosrun gps_rtk gps_rtk2" C-m
    
    tmux split-window -h
    tmux send-keys "sleep 15" C-m                                       #no needs to start earlier because 10/15 sec of calibration
    tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
    tmux send-keys "cd /home/pi/bagfiles" C-m
    tmux send-keys "rosbag record -a" C-m
    
    
elif [ "$#" -eq 5 ]
then
    echo "Here we go WITHOUT log files"
    sleep 2
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
    tmux send-keys "rosrun navio2_remote remote_multiCtr $1 $2 $3 $4 $5" C-m
elif [ "$#" -eq 6 ] && [ "$6" == '-log' ]          #to active the writing of the log files
    then
        echo "Here we go WITH log files"
        sleep 2
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
        tmux send-keys "rosrun navio2_remote remote_multiCtr $1 $2 $3 $4 $5" C-m
        tmux split-window -v
        tmux send-keys "sleep 10" C-m                                       #no needs to start earlier because 10/15 sec of calibration
        tmux send-keys "source /home/pi/catkin_ws/devel/setup.bash" C-m
        tmux send-keys "cd /home/pi/bagfiles" C-m
        tmux send-keys "rosbag record -a" C-m
else
    echo "Usage :   ./rosCustom.bash [freq] [MaxThrottlePwm] [Kp] [Ki] [Kd] [-log], -log is not necessary" #MaxThrottlePwm is limited to 2000 in the cpp
    echo "Or :      ./rosCustom.bash auto : for automatic mode i.e. freq= 50, MaxThrottlePwm=1600, Kp=0.7, Ki=0.2, Kd=0.2"
    exit 0
fi

# Set default window
tmux select-window -t $SESSION:1

# Attach to session
tmux -2 attach-session -t $SESSION

