# Project for the Robotics, Vision & AI course

Objective: solve the rubik cube with the YuMi robot

## Steps
  
  ### Calibration:
  - 

   
  ### Localization:
  Using a camera in the Coppelia simulation, localize the rubik cube
  - Create a cube in the scene, and add the camera (make sure the cube is centered w.r.t the camera's FOV)
  - Creat a topic /cube_pose that publish the cube's position and oreintation (use LUA script)
  - Create a topic /image that publish what the camera is seeing
  - Trajectory planning to reach the cube knowing its pose

   
  ### Pick

   
  ### Scanning:
  Scan each face to detect the configurations. Given the photos of the cube, make a script to detect the configuration on each face.

  
7) Execute the sequence of moves: use rubik solver to execute the moves according to the configuration

   
8) check that the cube is solved: check that each face contain just one color

   
9) drop the cube, bring robot in the starting position: open gripper, then trajectory planning to move robot to its original position

