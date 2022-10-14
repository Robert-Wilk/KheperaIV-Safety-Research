# KheperaIV-Safety-Research
CBF, SBC, and PrSBC research on Khepera IV robots using CoppeliaSim


This code utilizes the KheperaIV which is not native to CoppeliaSim. To add the KheperaIV robot to CoppeliaSim head to [this Github repo](https://github.com/EAPH/K4_Model_VREP), download the files, and import into CoppeliaSim. To learn more about the KheperaIV robot model in CoppeliaSim check out [Farias et al.'s research paper](https://doi.org/10.1016/j.ifacol.2017.08.1721).


This code also utilizes snippets from the [Robotarium Python Simulator](https://github.com/robotarium/robotarium_python_simulator).

### How to use:
1. Download CoppeliaSim
1. Download KheperaIV robot model and import into CoppeliaSim
2. Download this Repo
3. Import the scene you choose into CoppeliaSim
4. Run any of the Python files inside the "code" folder that contain \*_controller\*

### Current Issues:
- Robots might spin in some scripts before everything is fully initialized in the simulator
- Robots might not fully stop after given command to stop
- multi_controller_CBF.py doesn't generalize well to 8 robots
- multi_controller_PrSBC.py is not currently implemented
