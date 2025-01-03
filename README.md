# Overview

Depth estimation algorithm for neuromorphic-vision based roller tactile sensor (based on EMVS)

# Installation

Install [catkin tools](http://catkin-tools.readthedocs.org/en/latest/installing.html), [vcstool](https://github.com/dirk-thomas/vcstool):

    sudo apt-get install python-catkin-tools python-vcstool
    
Create a new catkin workspace if needed (replace {ros-version} with your ros version name e.g: noetic, kinetic):

    mkdir -p ~/roller_ws/src && cd ~/roller_ws/
    catkin config --init --mkdirs --extend /opt/ros/{ros-version} --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

Clone this repository:

    cd src/
    git clone [git@github.com:uzh-rpg/rpg_emvs.git](https://github.com/akramekhairi/NV_roller.git)

Clone dependencies:

    vcs-import < rpg_emvs/dependencies.yaml

Install `pcl-ros` ((replace {ros-version} with your ros version name e.g: noetic, kinetic)):

    sudo apt-get install ros-{ros-version}-pcl-ros

Build the package(s):

    catkin build mapper_emvs
    source ~/roller_ws/devel/setup.bash

# Running example

Download [stairs.bag](http://rpg.ifi.uzh.ch/datasets/davis/slider_depth.bag) data file

**Run the example**:

    roscd mapper_emvs
    rosrun mapper_emvs run_emvs --bag_filename=/path/to/slider_depth.bag --flagfile=cfg/slider_depth.conf


# Visualization


## Disparity Space Image (DSI)

We also provide Python scripts to inspect the DSI (3D grid).

### Volume Rendering

Install visvis first:

    pip install visvis

To visualize the DSI stored in the `dsi.npy` file, run:

    roscd mapper_emvs
    python scripts/visualize_dsi_volume.py -i /path/to/dsi.npy

You should get the following output, which you can manipulate interactively:

<img src="mapper_emvs/images/slider_depth/dsi_volume.gif" width="60%">

### Showing Slices of the DSI

To visualize the DSI with moving slices (i.e., cross sections), run:

    python scripts/visualize_dsi_slices.py -i /path/to/dsi.npy

which should produce the following output:

<img src="mapper_emvs/images/slider_depth/dsi_slice.gif" width="60%">

## Point Cloud

To visualize the 3D point cloud extracted from the DSI, install `pypcd` first as follows:

    pip install pypcd

and then run:

    python scripts/visualize_pointcloud.py -i /path/to/pointcloud.pcd
    
A 3D matplotlib interactive window like the one below should appear, allowing you to inspect the point cloud (color-coded according to depth with respect to the reference view):
    
<img src="mapper_emvs/images/slider_depth/pointcloud.gif" width="40%">

            
