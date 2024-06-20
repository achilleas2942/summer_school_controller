# AEROTRAIN SUMMER SCHOOL - Human-Robot Interaction Day: Controller with Delay Compensation
This work is a part of the AERO-TRAIN Summer School 2024. It was presented during Day 3 - Human-Robot Interaction Day, and it is part of [Tutorial 1](https://github.com/AERO-TRAIN/exercises_summer_school_hri_day/tree/main).

## Repository Description
- This repository is designed to run within a [docker container](https://github.com/achilleas2942/summer_school_controller/tree/main/docker) as a part of the [Tutorial 1](https://github.com/AERO-TRAIN/exercises_summer_school_hri_day/tree/main), howerver it can also run locally.
- The repository includes:
  - The [velocity controller](https://github.com/achilleas2942/summer_school_controller/blob/main/src/velocity_controller.py) for the _pelican_ UAV from the [rotors_simulator](https://github.com/ethz-asl/rotors_simulator).
  - The [keyboard interface](https://github.com/achilleas2942/summer_school_controller/blob/main/src/keyboard_teleoperation.py) to enable keyboard desired positions for the UAV.
  - The [position predictor](https://github.com/achilleas2942/summer_school_controller/blob/main/src/position_prediction.py) to estimate the actual position of the UAV by compensating for the average measured delays.
  - UDP tunnels to transmit the ROS messages (_odometry_ and _command_ messages) to any machine without communication constrains. In order for the UDP tunnels to work correctly, you should specify:
    - The correct reading IP and port in the [odometry_server.py](https://github.com/achilleas2942/summer_school_controller/blob/main/src/odometry_server.py) file
    - The correct destination IP and port in the [command_client.py](https://github.com/achilleas2942/summer_school_controller/blob/main/src/command_client.py) file
   
**NOTE:** The [main branch](https://github.com/achilleas2942/summer_school_controller/tree/main) includes unfinished codes where you can try to include your own solutions. indicative solutions are included in the [solutions branch](https://github.com/achilleas2942/summer_school_controller/tree/solutions)

### Dependencies
- [mav_comm](https://github.com/ethz-asl/mav_comm)
  ```
  git clone https://github.com/ethz-asl/mav_comm.git
  ```
  
### Clone the repository
  ```
  git clone https://github.com/achilleas2942/summer_school_controller.git
  ```
    
### Run the Gazebo world locally
- Follow the description of [summer_school_world](https://github.com/achilleas2942/summer_school_world) to run the Gazebo world.
- After building, e.g.
  ```
  catkin build summer_school_controller
  ```
  or
  ```
  catkin_make summer_school_controller
  ```
- And sourcing your workspace, e.g.
  ```
  source <your_workspace_name>/devel/setup.bash
  ```
- Execute the launch file to run the UDP tunnels
  ```
  roslaunch summer_school_controller udp_tunnels.launch
  ```
- Develop your code for the [velocity controller](https://github.com/achilleas2942/summer_school_controller/blob/main/src/velocity_controller.py) or use the script from the [solutions branch](https://github.com/achilleas2942/summer_school_controller/blob/solutions/src/velocity_controller.py) and run it
  ```
  python3 velocity_controller.py
  ```
- Develop your code for the [position prediction](https://github.com/achilleas2942/summer_school_controller/blob/main/src/position_prediction.py) or use the script from the [solutions branch](https://github.com/achilleas2942/summer_school_controller/blob/solutions/src/position_prediction.py) and run it
  ```
  python3 position_prediction.py
  ```
- Develop your code for the [keyboard teleoperation](https://github.com/achilleas2942/summer_school_controller/blob/main/src/keyboard_teleoperation.py) or use the script from the [solutions branch](https://github.com/achilleas2942/summer_school_controller/blob/solutions/src/keyboard_teleoperation.py) and run it
  ```
  python3 keyboard_teleoperation.py
  ```

## Troubleshooting
Consider opening an Issue if you have [troubles](https://github.com/achilleas2942/summer_school_controller/issues) with the exercises of the repo.\
Feel free to use the [Discussions](https://github.com/achilleas2942/summer_school_controller/discussions) tab to exchange ideas and ask questions.
