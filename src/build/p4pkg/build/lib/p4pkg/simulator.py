from p4pkg.disc_robot import load_disc_robot
from p4pkg.world_reader import read_world


def main():
    robot = load_disc_robot("normal.robot")
    l = robot['wheels']['distance'] #distance between robot's wheels

    world = read_world('brick.world') #tuple (resolution, pose, map)
    print(type(world[2]))
    print(world[2])




if __name__ == '__main__':
    main()
