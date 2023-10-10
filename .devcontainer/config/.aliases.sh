#! /bin/zsh

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
alias rviz='rviz2 -d ~/colcon_ws/src/naoqi_driver2/share/pepper.rviz'

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
}

alias foxy=foxy_f
alias colws='cd $COLCON_WS'
alias ws='cd $WS'
alias ros_setup='${WS}/setup.sh'
alias ros_build='${WS}/build.sh'
# ROS2 commands completion
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"
# shellcheck source=/dev/null
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.zsh

# launch pepper with specific ip

pepper() {
        if [ -z "$1" ]; then
                echo "No argument supplied"
                echo "Usage: pepper <ip>"
        else
                echo "Launching pepper with ip: $1"
                ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=$1
        fi
}

map() {
        if [ -z "$1" ]; then
                echo "No argument supplied"
                echo "Usage: ip_address:= <ip>"
        else
                ros2 launch pepper_nav rtabmap.launch.py ip_address:=$1
        fi
}

alias nav='ros2 launch pepper_nav navigation.launch.py'
