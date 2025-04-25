#!/bin/bash

# Send kill signal to each pane to give them a chance to shutdown gracefully
tmux send-keys -t phyto-arm:0.0 C-c
tmux send-keys -t phyto-arm:0.1 C-c
# Uncomment if running a second arm
# tmux send-keys -t phyto-arm:0.2 C-c

sleep 2

tmux kill-session -t phyto-arm
