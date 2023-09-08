#!/bin/bash

# Custom aliases for container internal user's shell

alias zshconfig="code ~/.zshrc"
alias ohmyzsh="code ~/.oh-my-zsh"
alias p10k="code ~/.p10k.zsh"
alias update="sudo apt-get update && sudo apt full-upgrade -y"
alias cat='batcat'
alias ls="lsd --group-dirs=first -S"
alias ll="ls -l --total-size -h"
alias la="ll -a"
alias lt="ls --tree --depth=2 "
alias clc='clear'
alias aliases='code ~/.aliases.sh'
alias status='git status'
alias pip='python3 -m pip'
alias movebot='rosrun rqt_robot_steering rqt_robot_steering'
# alias movejoint='rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller'
# alias rviz='rosrun rviz rviz -d ~/workspace/src/rviz_config/pepper.rviz'

# ROS2
foxy_f() {
        #echo "Sourcing ROS2 Foxy"
        local setup="$ROS2_INSTALL_PATH/setup.zsh"
        # shellcheck source=/dev/null
        source "$setup"
        local setup_ws_file="$COLCON_WS/install/setup.zsh"
        if [ -e "$setup_ws_file" ]; then
                # shellcheck source=/dev/null
                source "$setup_ws_file"
        fi
        eval "$(register-python-argcomplete3 ros2)"
        eval "$(register-python-argcomplete3 colcon)"
}

alias foxy=foxy_f
alias colws='cd $COLCON_WS'
alias ws='cd $WS'
alias ros_setup='${WS}/setup.sh'
alias ros_build='${WS}/build.sh'
