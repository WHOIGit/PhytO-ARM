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
  # Uncomment if a second arm is to be launched, e.g. arm_chanos.
  # Also uncomment commands below and last line in tmux_kill.sh
  # tmux split-window -v
  tmux select-layout even-vertical

  # Launch the main process in docker in pane 0
  tmux send-keys -t phyto-arm:0.0 "./scripts/docker_run.sh -b $CONFIG" C-m
  tmux send-keys -t phyto-arm:0.0 "./phyto-arm start main mounted_config.yaml" C-m

  # Launch the IFCB arm in pane 1
  tmux send-keys -t phyto-arm:0.1 "sleep 8" C-m
  tmux send-keys -t phyto-arm:0.1 "docker exec -it phyto-arm bash" C-m
  tmux send-keys -t phyto-arm:0.1 "./phyto-arm start arm_ifcb ./mounted_config.yaml" C-m

  # Select pane 3 and launch the Chanos arm in the same container
  # tmux send-keys -t phyto-arm:0.2 "sleep 10" C-m
  # tmux send-keys -t phyto-arm:0.2 "docker exec -it phyto-arm bash" C-m
  # tmux send-keys -t phyto-arm:0.2 "./phyto-arm start arm_chanos ./mounted_config.yaml" C-m

  # Attach to the session
  tmux attach -t phyto-arm
fi
