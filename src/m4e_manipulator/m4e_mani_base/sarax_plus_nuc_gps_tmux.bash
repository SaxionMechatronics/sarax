#!/bin/bash

# You need to have TMUX installed (sudo apt-get install tmux)
# Check: https://tmuxcheatsheet.com/

# Start a new TMUX session with a window named 'physical-test'
tmux new-session -d -s physical-test -n window

# Split the window into two horizontal panes
tmux split-window -h

# Select the first pane
tmux select-pane -t 0
# Split window into two vertical panes
tmux split-window

# Select the first pane
tmux select-pane -t 0
# Run the first program in the first pane
tmux send-keys "roslaunch mavros px4.launch" C-m
# Select the second pane
tmux select-pane -t 1
# Run the second program in the second pane
tmux send-keys "rosbag record -a"

# Select the 3rd pane
tmux select-pane -t 2
# Split window into two vertical panes
tmux split-window
# Select the 3rd pane
tmux select-pane -t 2
# Run the 3rd program in the 3rd pane
tmux send-keys "./start_physical_exp_gps_nuc.sh" 
# Select the 4th pane
tmux select-pane -t 3
# Run the 4th program in the 4th pane
tmux send-keys "roslaunch interaction_controller traj_generator.launch"
tmux select-pane -t 2

# Attach to the TMUX session to view the window
tmux attach-session -t physical-test

# To kill all windows: press (Ctrl+B) and then press (&) and then (y)
# To detattach the session and keep the processes running in the backgroud: 
# press (Ctrl+B) and then press (d)

# Check: https://tmuxcheatsheet.com/
