# Project for the Robotics, Vision & AI course

Steps
1) Calibration: perform hand-eye calibration of the camera mounted on top of the YuMi robot --> save calibration.yaml file
2) Localize the cube in the simulation: by building a function that fit planes
3) Pick the cube and move it to the target position: trajectory planning to pick the cube --> open gripper --> trajectory planning to move end effector to the target position
4) Scan the faces: scan all 6 faces and detect the configuration
5) Execute the sequence of moves: use rubik solver to execute the moves according to the configuration
6) check that the cube is solved: check that each face contain just one color
7) drop the cube, bring robot in the starting position: open gripper, then trajectory planning to move robot to its original position

