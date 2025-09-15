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
    git clone https://github.com/akramekhairi/NV_roller.git --branch rl_ycb

Clone dependencies:

    vcs-import < NV_roller/dependencies.yaml

Install `pcl-ros` ((replace {ros-version} with your ros version name e.g: noetic, kinetic)):

    sudo apt-get install ros-{ros-version}-pcl-ros

Build the package(s) and source them:

    cd ..
    catkin build mapper_emvs
    source devel/setup.bash

# Running example with mustard_slow.bag

**Run the example** (Filling in all parameters in {}. --chunk_size and --step_size are optional - defaults are 75000 and 5000 events respectively):

    roscd mapper_emvs
    python scripts/generate_increment.py --bag_filename bags/{bag_name.bag} --output_dir {output_dir} --flagfile cfg/{config.conf}
You should have the saved pointclouds, confidence maps, and respective depth maps within your specified {output_dir}





# Visualization

## Point Cloud

To visualize the 3D point cloud extracted from the DSI, install `open3d` first as follows:

    pip install open3d

and then run (remove -i /path/to/pointcloud.pcd to run the most recently computed pointcloud):

    python scripts/visualize_pointcloud.py -i /path/to/pointcloud.pcd
    

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

            
