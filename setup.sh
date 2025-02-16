#! /bin/zsh

set -e

create_missing_links() {
    local source_dir="$1"
    local destination_dir="$2"
    local filename
    # Check if the source directory is empty.
    # Dotfiles are ignored as we assume there are only packages in the source directory.
    if [ -z "$(ls "$source_dir")" ]; then
        echo "Source ($source_dir) directory is empty. No links will be created."
        return
    fi

    # Iterate over each file in the source directory
    for file in "$source_dir"/*; do
        # Get the file name
        filename=$(basename "$file")
        # Check if a symbolic link already exists for the file in the destination directory
        # or if a file with the same name exists in the destination directory
        if [ ! -e "$destination_dir/$filename" ]; then
            # Create a symbolic link in the destination directory if it doesn't exist
            ln -s "$file" "$destination_dir/$filename"
            echo "Created symbolic link for $source_dir/$filename on $destination_dir"
        else
            echo "Package or link already exists for $source_dir/$filename on $destination_dir"
        fi
    done
}

setup() {
    local setup="$ROS2_INSTALL_PATH/setup.zsh"
    local setup_ws_file="$COLCON_WS/install/setup.zsh"
    local install_command="cd ${COLCON_WS} && vcs import < src/ros2.repos src\
            && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS2_DISTRO}"
    if [ -e "$setup_ws_file" ]; then
        # shellcheck source=/dev/null
        env -i SHELL=/bin/zsh zsh -c "source $setup; source $setup_ws_file && $install_command"
    else
        # shellcheck source=/dev/null
        env -i SHELL=/bin/zsh zsh -c "source $setup && $install_command"
    fi
}

# Create symlinks for colcon_ws
create_missing_links "${WS}/src" "${COLCON_WS}/src/"

# update lists
printf "\nUpdating lists...\n"
sudo apt-get update >>/dev/null
rosdep update --include-eol-distros >>/dev/null
printf "\nInstalling dependencies...\n"

# assert that ros2 dependencies are downloaded, and install them
printf "\nInstalling ROS2 dependencies\n"
#env -i SHELL=/bin/zsh zsh -c "source ${ROS2_INSTALL_PATH}/setup.zsh; source ${COLCON_WS}/install/setup.zsh \
#&& cd ${COLCON_WS} && vcs import < src/ros2.repos src\
#&& rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS2_DISTRO}"
setup
