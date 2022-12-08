# KheperaIV-Safety-Research
SBC and PrSBC research on Khepera IV robots in simulated and real-world environments


This code utilizes the KheperaIV which is not native to CoppeliaSim. To add the KheperaIV robot to CoppeliaSim head to [this Github repo](https://github.com/EAPH/K4_Model_VREP), download the files, and import into CoppeliaSim. To learn more about the KheperaIV robot model in CoppeliaSim check out [Farias et al.'s research paper](https://doi.org/10.1016/j.ifacol.2017.08.1721).


This code also utilizes snippets from the [Robotarium Python Simulator](https://github.com/robotarium/robotarium_python_simulator) and [Swarm Setup from CMU](https://github.com/michael5511b/CMU-AART-Swarm-Platform-ROS-Package).

### How to use Simulation Scripts:
1. Download CoppeliaSim
2. Download KheperaIV robot model and import into CoppeliaSim
3. Download this Repo
4. Import the scene you choose into CoppeliaSim
5. Run any of the Python files inside the "code" folder that contain \*_controller\*

### How to use Real-World Scripts:
1. Download this Repo
2. Add the khepera_communicator (Real-World Code/khepera_communicator) ROS package to an existing ROS workspace
3. catkin_make the workspace
4. Download/configure/run either vicon_bridge package or vrpn_client_ros package
5. ssh into each KheperaIV robot and run the template file (Real-World Code/) using ./template
6. Run K4_Send_Cmd.py to establish robot-computer communication
7. Run K4_Mailbox.py to aggregates the Optitrack data for every robot into a single message
8. Run any of the Central_Algorithm_*.py files using ROS 


