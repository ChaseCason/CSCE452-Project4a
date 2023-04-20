# The load_disc_robot method reads a file that
# describes a disc-shaped robot and returns a
# dictionary describing the properties of that robot.

import yaml
import os

dirname = os.path.dirname(__file__)

def read_world(file_name):
    with open(os.path.join(dirname, file_name)) as f:
        world = yaml.safe_load(f)
    
    resolution = world['resolution'] #length of each side of block
    initial_pose = world['initial_pose'] #initial robot position in meters
    world_map = world['map'] #
    parsed_map = []

    

    return (resolution, initial_pose, world_map) 
