# consensus

consensus is ROS package used to test cooperative controllers with mobile robots in the Laboratory of Automation and Robotics at University of Brasilia.

The code in this package can be used either with simulated robots or with real robots:

## 1. Prerequisites

* This software was tested in Ubuntu 16.04 and 18.04
* (ROS Kinetic)[https://wiki.ros.org/kinetic/Installation/Ubuntu] and (ROS Melodic)[https://wiki.ros.org/melodic/Installation/Ubuntu]
* Gazebo 9.0.0

## 2. Build on ROS

Clone this repository to your catkin workspace.

```bash
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Gastd/consensus.git
  cd ../
  catkin_make
```


## 3. Run the Simulation
The Simulation use the well known Gazebo Simulated that is shiped with ROS. 

* Before running the simulation, you must to download our robots description.

```bash
    cd ${YOUR_WORKSPACE_PATH}/src
    git clone https://github.com/lara-unb/aramis.git
    git clone https://github.com/lara-unb/athos.git
    git clone https://github.com/lara-unb/porthos.git
    cd ../
    catkin_make
```

Then you can test the code, just run the simulation:

* In a terminal with your ros workspace
    ```bash
        roslaunch consenus simulation.launch
    ```
This will open the Gazebo with the robots in posistion ready to go.


## 2.Algorithms
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
    ```bash
        roslaunch consensus sim_flocking.launch
    ```

And the `flocking.launch` will run only a controller and must be used in the robot's computer.
The `display.launch` will open the RViz and will show the robots and its path when they move.
The `flocking_leader.py` implements the _virtual_ leader and can be used with `rosrun`:
    ```bash
    rosrun consensus flocking_leader.py
    ```


* `double_integrator.py` implements
