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



# RUBIK CUBE COLOR RECOGNITION

## GENERAL PROCESS
* **Objective:** The purpose of this Python script is to accurately detect the colors of each face of the Rubik's cube, enabling the robot to compute the optimal sequence of moves required to solve it.
* **Image Acquisition & Grid Generation:** Images of all six faces are imported (e.g., in `.jpg`/`.jpeg` format). The script isolates the cube by cropping the center of each image and overlays a $3 \times 3$ grid to locate the 9 individual stickers per face.
* **Color Processing & Optimization:** To ensure robust color recognition under varying ambient lighting conditions, pixel data is sampled from the median color of each grid cell and converted from BGR to the **CIELAB ($L^*a^*b^*$) color space**. This data can be visualized in a 3D scatter plot using the actual RGB coordinates, while a linear assignment algorithm (Hungarian Method) guarantees that exactly 9 stickers are assigned to each of the 6 colors.

---

## CUBE ROTATIONS & SCANNING PROCESS

The robot scans each face of the cube sequentially using a dual-gripper system. The routine begins by securing the cube with the right hand, scanning the first three faces via the right hand, and then transitioning control to the left hand for the remaining faces.

### 1. RIGHT HAND SEQUENCE 
* **Front Face:** The scanning process begins with the camera capturing the Front face.
* **Left Face:** The robot's right gripper rotates 90° to the right to expose the Left face to the camera.
* **Back Face:** The gripper returns to its initial position and executes a 180° rotation to scan the Back face.

### 2. LEFT HAND SEQUENCE
* **Top (Up) Face:** The left hand takes control, clamping the cube and rotating it 90° forward to scan the Top face.
* **Right Face:** A 90° rotation is made to the left to expose the Right face.
* **Bottom (Down) Face:** Finally, the cube is flipped by 180° from its top position to scan the Bottom face.

### Unwrapped Cube Layout
The diagram below illustrates how the physical sequence of mechanical motions maps onto a flat, unwrapped representation of the cube:

```
       +-------+
       | FRONT |
+------+-------+-------+--------+
| LEFT |  UP   | RIGHT | BOTTOM |
+------+-------+-------+--------+
       | BACK  |
       +-------+
```
---

### ⚠️ CRITICAL NOTE ON ORIENTATION
Because the physical robot rotates the cube along different axes during the scanning process, several faces are captured sideways or upside-down relative to the camera's fixed frame of reference. 

For example, the **Back face** is captured with a 180° orientation shift, meaning its color data is read upside-down. To compensate for this, the script mathematically rotates the internal $3 \times 3$ color matrices (e.g., remapping array indices) based on each face's specific perspective before compiling the final 54-character state string required by the Kociemba solver.



