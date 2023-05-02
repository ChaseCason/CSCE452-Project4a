# The load_disc_robot method reads a file that
# describes a disc-shaped robot and returns a
# dictionary describing the properties of that robot.

import yaml
import os

from nav_msgs.msg import OccupancyGrid

dirname = os.path.dirname(__file__)

def read_world(file_name):
    with open(os.path.join(dirname, file_name)) as f:
        world = yaml.safe_load(f)
    
    resolution = world['resolution'] #length of each side of block
    initial_pose = world['initial_pose'] #initial robot position in meters
    world_map = world['map']
    parsed_map = []

    for row in world_map.strip().split('\n'):
        currRow = []
        for gridElement in row.strip():
            if gridElement == '.':
                currRow.append(0)
            elif gridElement == '#':
                currRow.append(100)
            else:
                raise ValueError("Invalid character")
        parsed_map.insert(0,currRow)

    msg = OccupancyGrid()
    msg.header.frame_id = "world"
    msg.info.width = len(parsed_map[0])
    msg.info.height = len(parsed_map)
    msg.info.resolution = resolution
    msg.data = [val for row in parsed_map for val in row]

    return (msg, initial_pose)
