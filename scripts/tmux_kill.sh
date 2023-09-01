#!/bin/bash

# Send kill signal to each pane to give them a chance to shutdown gracefully
tmux send-keys -t 0 C-c
tmux send-keys -t 1 C-c
tmux send-keys -t 2 C-c

sleep 2

tmux kill-session -t phyto-arm
