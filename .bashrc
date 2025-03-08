## To setup run:
# mv ~/.bashrc ~/.bashrc.bak
# ln -s ~/AMR-tiago/.bashrc ~/.bashrc 

# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# Fresh rebuild for ROS 2 and Tiago workspaces
rebuild-ws() {
    # Rebuild ros2_ws
    echo "Rebuilding ros2_ws..."
    cd ~/AMR-tiago/ros2_ws || { echo "Error: ros2_ws directory not found." >&2; return 1; }
    trash build install log  # Clean existing build artifacts
    colcon build || return 1  # Build the workspace
    
    # Rebuild tiago_ws
    echo "Rebuilding tiago_ws..."
    cd ~/AMR-tiago/tiago_ws || { echo "Error: tiago_ws directory not found." >&2; return 1; }
    trash build install log  # Clean existing build artifacts
    colcon build || return 1  # Build the workspace
    
    echo "Both workspaces rebuilt successfully!"
    cd ~  # Return to home directory
}

rebuild-AMR-tiago() {
    # Rebuild AMR-tiago
    echo "Rebuilding AMR-tiago..."
    cd ~/AMR-tiago/exam_ws || { echo "Error: AMR-tiago directory not found." >&2; return 1; }
    trash build install log  # Clean existing build artifacts
    colcon build || return 1  # Build the workspace
    
    echo "AMR-tiago workspace rebuilt successfully!"
    cd ~  # Return to home directory
}

# Fresh rebuild for ROS 2 and Tiago workspaces
build-ws() {
    # Rebuild ros2_ws
    echo "Building ros2_ws..."
    cd ~/AMR-tiago/ros2_ws || { echo "Error: ros2_ws directory not found." >&2; return 1; }
    colcon build || return 1  # Build the workspace
    
    # Rebuild tiago_ws
    echo "Building tiago_ws..."
    cd ~/AMR-tiago/tiago_ws || { echo "Error: tiago_ws directory not found." >&2; return 1; }
    colcon build || return 1  # Build the workspace
    
    echo "Both workspaces built successfully!"
    cd ~  # Return to home directory
}

build-AMR-tiago() {
    # Rebuild AMR-tiago
    echo "Building AMR-tiago..."
    cd ~/AMR-tiago/exam_ws || { echo "Error: AMR-tiago directory not found." >&2; return 1; }
    colcon build || return 1  # Build the workspace
    
    echo "AMR-tiago workspace built successfully!"
    cd ~  # Return to home directory
}

run-save-map() {
    version=$(date +%Y%m%d%H%M%S)
    mv ~/AMR-tiago/maps/map.pgm ~/AMR-tiago/maps/map_$version.pgm
    mv ~/AMR-tiago/maps/map.yaml ~/AMR-tiago/maps/map_$version.yaml
    ros2 run nav2_map_server map_saver_cli -f ~/AMR-tiago/maps/map
}

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias tmux_setup='bash ~/AMR-tiago/tmux/setup_tmux.sh'
alias run-gazebo='ros2 launch tiago_gazebo tiago_gazebo.launch.py group_number:=32 moveit:=True'
alias run-rviz-slam='ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=false rviz:=True slam:=True'
alias run-rviz='ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=false rviz:=True map_path:=/home/$USER/AMR-tiago/maps'
alias run-teleop='ros2 run teleop_twist_keyboard teleop_twist_keyboard'
alias run-explore-lite='ros2 launch explore_lite explore.launch.py'
alias run-navigate-to-pose='ros2 run tiago_exam_navigation navigate_to_pose'
alias run-target-locked='ros2 run tiago_exam_camera target_locked'
alias run-eyes='ros2 run tiago_exam_camera image_sub'
alias run-head-joystick='ros2 run tiago_exam_navigation move_head'
alias run-pick-and-place= 'ros2 run tiago_exam_arm 4_pick_and_place' 

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

#source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=30 #TURTLEBOT3
export LC_NUMERIC="en_US.UTF-8"
export TURTLEBOT3_MODEL=burger
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source ~/AMR-tiago/ros2_ws/install/setup.bash
source ~/AMR-tiago/tiago_ws/install/setup.bash
# source ~/AMR-tiago/tiago_ws/src/install/setup.bash
source ~/AMR-tiago/exam_ws/install/setup.bash