# KheperaIV-Safety-Research
CBF, SBC, and PrSBC research on Khepera IV robots in simulated and real-world environments


This code utilizes the KheperaIV which is not native to CoppeliaSim. To add the KheperaIV robot to CoppeliaSim head to [this Github repo](https://github.com/EAPH/K4_Model_VREP), download the files, and import into CoppeliaSim. To learn more about the KheperaIV robot model in CoppeliaSim check out [Farias et al.'s research paper](https://doi.org/10.1016/j.ifacol.2017.08.1721).


This code also utilizes snippets from the [Robotarium Python Simulator](https://github.com/robotarium/robotarium_python_simulator) and [Swarm Setup from CMU](https://github.com/michael5511b/CMU-AART-Swarm-Platform-ROS-Package).

### How to use:
1. Download CoppeliaSim
1. Download KheperaIV robot model and import into CoppeliaSim
2. Download this Repo
3. Import the scene you choose into CoppeliaSim
4. Run any of the Python files inside the "code" folder that contain \*_controller\*

### Current Issues (As of 11/17/2022):
- Circle Cross (8 robot test) runs slowly in CoppeliaSim



