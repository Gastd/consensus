# consensus

## Overview

consensus is ROS package used to test cooperative controllers with mobile robots in the Laboratory of Automation and Robotics at University of Brasilia.

The code in this package can be used either with **simulated robots** or with **real robots**:

### License

The source code is released under a [MIT license](LICENSE).

**Author: Gabriel F P Araujo<br />
Affiliation: [LARA](https://lara.unb.br/)<br />
Maintainer: Gabriel F P Araujo, gabriel.fp.araujo@gmail.com
            Raphael Braccialli, raphael.braccialli@gmail.com**

The consensus package has been tested under the following softwares:

* This software was tested in Ubuntu 16.04 and 18.04
* [ROS Kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) and [ROS Melodic](https://wiki.ros.org/melodic/Installation/Ubuntu)
* Gazebo 9.0.0

 This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) -->


<!-- ![Example image](doc/example.jpg) -->

## Build on ROS

Clone this repository to your catkin workspace.

```bash
cd ${YOUR_WORKSPACE_PATH}/src
git clone https://github.com/Gastd/consensus.git
cd ../
catkin_make
```


## Run the Simulation
The Simulation uses the well known Gazebo Simulated that is shiped with ROS. 

* Before running the simulation, you must to download our robots description.

    ```bash
    cd ${YOUR_WORKSPACE_PATH}/src
    git clone https://github.com/lara-unb/aramis.git
    git clone https://github.com/lara-unb/athos.git
    git clone https://github.com/lara-unb/porthos.git
    cd ../
    catkin_make
    ```

Run the simulation to test the code:

* In a terminal with your ros workspace
    ```
    roslaunch consenus simulation.launch
    ```

This will open the Gazebo with the robots in posistion and ready to go.


## Algorithms
* `flocking_leader.py` and `flocking_agents.py` implements the control system proposed in the following article:

    ```
    @article{zhao2017,
      title={Flocking of multi-agent nonholonomic systems with unknown leader dynamics and relative measurements},
      author={Zhao, Xiao-Wen and Guan, Zhi-Hong and Li, Juan and Zhang, Xian-He and Chen, Chao-Yang},
      journal={International Journal of Robust and Nonlinear Control},
      volume={27},
      number={17},
      pages={3685--3702},
      year={2017},
      publisher={Wiley Online Library}
    }
    ```

There is a launch file that calls the controllers for each one of the robots including the _virtual_ leader.
    
    roslaunch consensus sim_flocking.launch


And the `flocking_agents.launch` will run only one controller and must be used in each robot's computer.
The `display.launch` will open the RViz and will show the robots and its path when they move.
The `flocking_leader.py` implements the _virtual_ leader and can be used with `rosrun`:

    rosrun consensus flocking_leader.py


* `double_integrator.py` implements

<!-- ## Disclaimer

This code is used in research, have a good time! -->

<!-- # Package Name

## Overview

This is a template: replace, remove, and add where required. Describe here what this package does and what it's meant for in a few sentences.

**Keywords:** example, package, template

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: Péter Fankhauser<br />
Affiliation: [ANYbotics](https://www.anybotics.com/)<br />
Maintainer: Péter Fankhauser, pfankhauser@anybotics.com**

The PACKAGE NAME package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/)


![Example image](doc/example.jpg)


### Publications

If you use this work in an academic context, please cite the following publication(s):

* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2015. ([PDF](http://dx.doi.org/10.3929/ethz-a-010173654))

        @inproceedings{Fankhauser2015,
            author = {Fankhauser, P\'{e}ter and Hutter, Marco},
            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
            title = {{PAPER TITLE}},
            publisher = {IEEE},
            year = {2015}
        }


## Installation

### Installation from Packages

To install all packages from the this repository as Debian packages use

    sudo apt-get install ros-indigo-...

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

    sudo apt-get install libeigen3-dev


#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

  cd catkin_workspace/src
  git clone https://github.com/ethz-asl/ros_package_template.git
  cd ../
  catkin_make


### Unit Tests

Run the unit tests with

  catkin_make run_tests_ros_package_template


## Usage

Describe the quickest way to run this software, for example:

Run the main node with

  roslaunch ros_package_template ros_package_template.launch

## Config files

Config file folder/set 1

* **config_file_1.yaml** Shortly explain the content of this config file

Config file folder/set 2

* **...**

## Launch files

* **launch_file_1.launch:** shortly explain what is launched (e.g standard simulation, simulation with gdb,...)

     Argument set 1

     - **`argument_1`** Short description (e.g. as commented in launch file). Default: `default_value`.

    Argument set 2

    - **`...`**

* **...**

## Nodes

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

  The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

  Returns information about the current average. For example, you can trigger the computation from the console with

    rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

  The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

  The size of the cache.


### NODE_B_NAME

...


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
 -->
