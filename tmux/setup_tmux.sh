#!/bin/bash

###########################################
# Personal Configuration [Edit this section]
###########################################

# setup your ssh target and the position of the local ubm-f1tenth repo

# warning: the tmux pane numbers are 1-indexed
# see README.md for more information

# if you want to add more configurations the only thing you need to do are:
# 1) add an new entry to the CONFIGS array
# 2) add a new function that sets up the tmux session

declare -A CONFIGS=(
    ["mapping"]="gaezbo, rviz-SLAM, run-explore-lite",
    ["party_mode"]="gaezbo, rviz, target_locked, navigate_to_pose"
    ["arm"]="todo"
    # Add more configurations here
)

mapping(){
    
    # pane 1
    tmux send-keys C-l
    tmux send-keys "run-gazebo"
    tmux split-window -v -p 30  # Split vertically

    # pane 4
    tmux send-keys C-l
    tmux send-keys "run-save-map"

    # pane 2
    tmux select-pane -t 1
    tmux split-window -h -p 66  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "run-rviz-slam"

    # pane 3
    tmux split-window -h -p 50  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "run-explore-lite"

    # pane 5
    tmux select-pane -t 4
    tmux split-window -h -p 33
    tmux send-keys "micro $SETUP_TMUX_FOLDER/mapping_notes.txt" C-m

    #tmux new-window -n "tab name"
    tmux select-pane -t 1
}

party_mode(){
    
    # pane 1
    tmux send-keys C-l
    tmux send-keys "run-gazebo"
    tmux split-window -v -p 30  # Split vertically

    # pane 4
    tmux send-keys C-l
    tmux send-keys "ros2 run tiago_exam_navigation align_to_box_face"

    # pane 2
    tmux select-pane -t 1
    tmux split-window -h -p 66  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "run-rviz"

    # pane 3
    tmux split-window -h -p 50  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "run-nav-to-box"

    # pane 5
    tmux select-pane -t 4
    tmux split-window -h -p 33
    tmux send-keys "micro $SETUP_TMUX_FOLDER/party_mode_notes.txt" C-m

    #tmux new-window -n "tab name"
    tmux select-pane -t 1
}

arm(){
    
    # pane 1
    tmux send-keys C-l
    tmux send-keys "run-gazebo"
    tmux split-window -v -p 30  # Split vertically

    # pane 6
    tmux send-keys C-l
    tmux send-keys "run-aruco-grab-controller "

    # pane 2
    tmux select-pane -t 1
    tmux split-window -h -p 80  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "ros2 run tiago_exam_arm publish_aruco_cube"

    # pane 3
    tmux split-window -h -p 75
    tmux send-keys C-l
    tmux send-keys "run-grasp-pose"

    # pane 4
    tmux split-window -h -p 66
    tmux send-keys C-l
    tmux send-keys "run-move-arm"

    # pane 5
    tmux split-window -h -p 50  # Split horizontally
    tmux send-keys C-l
    tmux send-keys "run-pick-and-place"

    # pane 7
    tmux select-pane -t 6
    tmux split-window -h -p 33
    tmux send-keys "micro $SETUP_TMUX_FOLDER/arm_notes.txt" C-m

    #tmux new-window -n "tab name"
    tmux select-pane -t 1
}

# Add more functions here

###########################################
# Helper Functions
###########################################

show_help() {
    echo "Usage: $0 <configuration>"
    echo ""
    echo "Available configurations:"
    for config in "${!CONFIGS[@]}"; do
        printf "  %-10s %s\n" "- $config" "${CONFIGS[$config]}"
    done
}

validate_config() {
    local config=$1
    if [ -z "$config" ] || [ -z "${CONFIGS[$config]}" ]; then
        show_help
        exit 1
    fi
}

###########################################
# Main Script
###########################################

SESSION="AMR-project"
SETUP_TMUX_FOLDER=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Validate input
validate_config "$1"

# Check if tmux session already exists
tmux has-session -t $SESSION 2>/dev/null

# If session does not exist, create it
if [ $? != 0 ]; then
    tmux new-session -d -s $SESSION -n "$1" -x- -y-
    $1
else
    echo "Session already exists. Should I kill it and start a new one? [Y/n]"
    read -r response
    response=${response:-Y}  # Default to "Y" if no response is provided
    if [ "$response" == "Y" ] || [ "$response" == "y" ]; then
        tmux kill-session -t $SESSION
        tmux new-session -d -s $SESSION -n "$1"
        $1
    else
        echo "Exiting..."
        exit 0
    fi
fi

# Attach to session
tmux attach-session -t $SESSION
