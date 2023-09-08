# Pepper-Workspace

> based on althack/ros2-vscode-template [GitHub](https://github.com/athackst/vscode_ros2_workspace)

## Usage

Repository for a `ROS2(foxy) workspace` with `vscode` settings. Using `remote container`, once you open the workspace in vscode, it will automatically build the workspace and install the vscode extensions. Feel free to modify the `Dockerfiles` and other stuffs to your needs.

To build the container use VSCode remote-container extensions as follows:
1. `CTRL + SHIFT + P` to open the command list
2. Type in the textbox `open folder in container` and select the folder where you cloned the repo.
3. In the prompted menu, select the container that you want to build. Ther are options for NVIDIA and Intel GPUs support (read preprequisites in the section below)

## Prerequisites

To use the an NVIDIA GPU, you need to install the `NVIDIA Container Toolkit` and setup a proper runtime. More info about it in the [NVIDIA's offical documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).

If you want to use Intel GPU no prerequisite should be needed. As far as I know, this should be true for AMD graphics card either.

## How to use

The container is splitted in two workspaces. The one that you "can see" is made by every files that are in the repo's folder. This is possible because the local workspace is mounted in the container automatically by VSCode. The second workspace (ROS2 workspace inside the colcon_ws folder) lives inside the container, and can be explored as a normal file system using terminal.

If you want explore container's file system with VSCode just write in the terminal the command `code /`. This will open a new VSCode window where the root of the filesystem is mounted. 

To prevent accidental damage, instead of the root (`/`) foolder just type the path of the folder that you want to inspect. You can then navigate to the desired folder using the terminal and then use `code .` to get the same behavior. 

Fullpath of the folder can also be given to the `code` command if you want to open desired folder from everywhere. An alias for this could be set in the `/.devcontainer/config/.aliases.sh` file.

>the `setup.sh` file

This file is used to update ros repositories, install missing depencencies and makes symbolic links from the `local workspace/src` folder to the `home` folder. With this trick, you can modify the ROS2 workspace locally and build packages as if your files lives inside the container.

>the `build.sh` file

This file is used to build the workspace. Remember that this should be the only way to build your workspace, otherwise `colcon` will create `install`, `build`, and `log` folders in the folder where is called. If you call `colcon build` accidentally, remember to manually remove those directories.


> building pipepline

Recap for the pipeline that users should follow to build the workspace is the following:

1. run `./setup.sh` (or `ros_setup` alias) to update ros repositories, install missing depencencies and makes symbolic links.
2. run `./build.sh` (or `ros_build` alias) to build the `colcon workspace`.
