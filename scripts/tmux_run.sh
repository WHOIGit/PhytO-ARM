#!/bin/bash

# Default config path
CONFIG='configs/config.yaml'

# If an arg is passed in use that instead
if [ -n "$1" ]; then
      CONFIG=$1
fi


# Check if the tmux session exists
if tmux has-session -t phyto-arm 2>/dev/null; then
  # If it exists, attach to it
  tmux attach -t phyto-arm
else
  # Create the session and name the window
  tmux new-session -d -s phyto-arm -n docker
  
  # Split the window into three panes
  tmux split-window -v
  tmux split-window -v
  tmux select-layout even-vertical
  
  # Select pane 1 and launch the main process in docker
  tmux select-pane -t 0
  tmux send-keys "./scripts/docker_run.sh -b $CONFIG" C-m
  tmux send-keys "./phyto-arm start main mounted_config.yaml" C-m
  
  # Select pane 2 and launch the IFCB arm in the same container
  tmux select-pane -t 1
  tmux send-keys "sleep 8" C-m
  tmux send-keys "docker exec -it phyto-arm bash" C-m
  tmux send-keys "./phyto-arm start arm_ifcb ./mounted_config.yaml" C-m
  
  # Select pane 3 and launch the Chanos arm in the same container
  tmux select-pane -t 2
  tmux send-keys "sleep 10" C-m
  tmux send-keys "docker exec -it phyto-arm bash" C-m
  tmux send-keys "./phyto-arm start arm_chanos ./mounted_config.yaml" C-m
  
  # Attach to the session
  tmux attach -t phyto-arm
fi
