# consensus

consensus is ROS package used to test cooperative controllers with mobile robots in the Laboratory of Automation and Robotics at University of Brasilia.

The code in this package can be used either with simulated robots or with real robots:

## 1. Prerequisites

## 2. Build on ROS

## 3. Run the Simulation
The Simulation use the well known Gazebo Simulated that is shiped with ROS. The Code here was tested in  

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

* `double_integrator.py` implements
