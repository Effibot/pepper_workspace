#! /bin/zsh
set -e

build() {
        local setup="$ROS2_INSTALL_PATH/setup.zsh"
        local setup_ws_file="$COLCON_WS/install/setup.zsh"
        local skip_pkgs="nav2_bt_navigator nav2_amcl nav2_controller nav2_behavior_tree nav2_msgs nav2_planner nav2_costmap_2d nav2_dwb_controller nav2_bringup nav2_voxel_grid doc nav2_core nav2_utils tools nav2_map_server nav2_regulated_pure_pursuit_controller nav2_lyfecicle_manager nav2_waypoint_follower nav2_navfn_planner nav2_system_tests smac_planner nav2_common nav2_rviz_plugins nav2_recoveries navigation2 dwb_core dwb_critics dwb_msgs dwb_plugins costmap_queue nav2_gazebo_spawner"
        local build_args="--symlink-install --packages-skip $skip_pkgs --event-handlers console_direct+ --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'"
        if [ -e "$setup_ws_file" ]; then
                # shellcheck source=/dev/null
                env -i SHELL=/bin/zsh zsh -c "source $setup; source $setup_ws_file; \
                cd ${COLCON_WS} && colcon build $build_args"
        else
                echo "No workspace found, launching standard build"
                env -i SHELL=/bin/zsh zsh -c "source $setup; \
                cd ${COLCON_WS} && colcon build $build_args"
        fi
}

# build ROS2 workspace
echo "Building Foxy Workspace"
#env -i SHELL=/bin/zsh zsh -c "source /opt/ros/foxy/setup.zsh; source ~/colcon_ws/install/setup.zsh \
#&& cd ${COLCON_WS} && colcon build --merge-install --symlink-install --event-handlers console_direct+ \
#--cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On'"
build
