import os
from os.path import isfile
import numpy as np

def locate_cube(image):
    # This function would contain the logic to locate the cube in the image
    # For now, we will just return a placeholder value
    if isfile("cube_config.txt"):
        with open("cube_config.txt", "r") as file:
            config = file.readlines()
            # Convert the first line to a numpy array of floats
            cube_pose = np.array([float(x) for x in config[0].strip().split(",")])        
            print(f"Cube pose: {cube_pose}")

    return cube_pose

def scan_cube(image):
    # This function would contain the logic to locate the cube in the image
    # For now, we will just return a placeholder value
    if isfile("cube_config.txt"):
        with open("cube_config.txt", "r") as file:
            config = file.readlines()
            # read the third line starting form the first empty space and make it a string
            cube_config = config[2].strip().split(" ", 1)[1]
            print(f"Cube configuration: {cube_config}")


    return cube_config