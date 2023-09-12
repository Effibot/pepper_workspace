#! /bin/bash
set -e

build() {
        local setup="$ROS2_INSTALL_PATH/setup.zsh"
        local setup_ws_file="$COLCON_WS/install/setup.zsh"
        local build_args="--symlink-install --event-handlers console_direct+ \
                --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'"
        if [ -e "$setup_ws_file" ]; then
                # shellcheck source=/dev/null
                env -i SHELL=/bin/zsh zsh -c "source $setup; source $setup_ws_file; \
                cd ${COLCON_WS} && colcon build $build_args"
        else
                echo "No workspace found, launching standard build"
                env -i SHELL=/bin/zsh zsh -c "source $setup; \
                cd ${COLCON_WS} && colcon build $build_args"
        fi
        # source the local workspace again to get builded packages
        # shellcheck source=/dev/null
        source "$setup_ws_file"
}

# build ROS2 workspace
echo "Building Foxy Workspace"
#env -i SHELL=/bin/zsh zsh -c "source /opt/ros/foxy/setup.zsh; source ~/colcon_ws/install/setup.zsh \
#&& cd ${COLCON_WS} && colcon build --merge-install --symlink-install --event-handlers console_direct+ \
#--cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'"
build
