"""

A* grid planning

author: Atsushi Sakai(@Atsushi_twi)
        Nikos Kanargias (nkana@tee.gr)

See Wikipedia article (https://en.wikipedia.org/wiki/A*_search_algorithm)

"""

import math
import numpy as np

import os
import yaml
from ament_index_python.packages import get_package_share_directory

class AStarPlanner:

    def __init__(self, resolution, rr, padding):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.padding = padding
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()

        # custom parameters
        self.map_resolution = 1
        self.map_origin = (0, 0)


        self.resolution = self.map_resolution
        
        ox, oy = self.load_map()

        self.calc_obstacle_map(ox, oy)

        print("Map loading done!")

    def load_map(self):
        print("Loading map start!")
        map_yaml_file = os.path.join(get_package_share_directory('main_pkg'), 'map', 'home.yaml')
        map_yaml_data = yaml.full_load(open(map_yaml_file))

        self.map_resolution = map_yaml_data['resolution']    # m / pixel
        self.map_origin = map_yaml_data['origin']    # list
        

        map_pgm_file = os.path.join(get_package_share_directory('main_pkg'), 'map', map_yaml_data['image'])

        with open(map_pgm_file, 'rb') as pgmf:
            pgm_data = pgmf.readlines()
            map_width, map_height = map(int, pgm_data[1].split())
            map_data = np.array(list(map(int, pgm_data[3])))
            map_data[map_data <= 210] = 0
            map_data[map_data > 210] = 100
            map_data = map_data.reshape((map_height, map_width))
            # map_data = np.rot90(map_data, 3)
            map_data = np.flip(map_data, axis=0)    # 이상하게 불러온 맵이 플립되어있음

        # set obstacle positions
        ox, oy = [], []
        padded_map_data = map_data.copy()
        for i in range(map_height):
            for j in range(map_width):
                if map_data[i][j] == 0:
                    ox.append(j)
                    oy.append(i)
                    for dx in range(-self.padding, self.padding + 1):
                        for dy in range(-self.padding, self.padding + 1):
                            if 0 <= j + dx < map_width and 0 <= i + dy < map_height and padded_map_data[i + dy][j + dx] != 0:
                                padded_map_data[i + dy][j + dx] = 0
                                ox.append(j + dx)
                                oy.append(i + dy)

        return ox, oy

    class Node:
        def __init__(self, x, y, cost, parent_index, vector = None):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index
            self.vector = vector

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx_real, sy_real, gx_real, gy_real):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        sx = (sx_real - self.map_origin[0]) / self.map_resolution
        sy = (sy_real - self.map_origin[1]) / self.map_resolution
        gx = (gx_real - self.map_origin[0]) / self.map_resolution
        gy = (gy_real - self.map_origin[1]) / self.map_resolution
        
        print(sx_real, sy_real, gx_real, gy_real)
        print(sx, sy, gx, gy)

        
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                # key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
                key=lambda o: open_set[o].cost + self.calc_manhattan(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                goal_node.vector = current.vector
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):

                is_turned = 0
                before_vector = (0, 0)
                now_vector = (self.motion[i][0], self.motion[i][1])
                if closed_set.get(current.parent_index):
                    before_node = closed_set[current.parent_index]
                    before_vector = (current.x - before_node.x, current.y - before_node.y)
                    is_turned = before_vector != now_vector
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2] * (1 + is_turned), c_id, now_vector)
                
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry, tpx, tpy, tvec_x, tvec_y = self.calc_final_path(goal_node, closed_set)


        # grid position to map position
        for i in range(len(tpx)):
            tpx[i] = (tpx[i] * self.map_resolution) + self.map_origin[0]
            tpy[i] = (tpy[i] * self.map_resolution) + self.map_origin[1]

        print(tpx, tpy, tvec_x, tvec_y)

        return rx, ry, tpx, tpy, tvec_x, tvec_y

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        tpx, tpy = [], []
        tvec_x, tvec_y = [], []
        
        parent_index = goal_node.parent_index
        now_node = goal_node
        before_vector = (0, 0)
        now_vector = (0, 0)
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))

            is_turned = 0
            now_vector = (n.x - now_node.x, n.y - now_node.y)
            is_turned = now_vector != before_vector
        
            if is_turned:
                tpx.append(now_node.x)
                tpy.append(now_node.y)
                tvec_x.append(now_node.vector[0])
                tvec_y.append(now_node.vector[1])

            parent_index = n.parent_index
            now_node = n
            before_vector = now_vector
        
        return rx, ry, tpx[::-1], tpy[::-1], tvec_x[::-1], tvec_y[::-1]

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d
    
    @staticmethod
    def calc_manhattan(n1, n2):
        d = abs(n1.x - n2.x) + abs(n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False


        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):
        print("Calc Obstacle...")

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            # [-1, -1, math.sqrt(2)],
            # [-1, 1, math.sqrt(2)],
            # [1, -1, math.sqrt(2)],
            # [1, 1, math.sqrt(2)]
        ]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = 0  # [m]
    sy = 0  # [m]
    gx = -0.16069  # [m]
    gy = -1.9620166  # [m]
    grid_size = 0.7  # [m]
    robot_radius = 0.3  # [m]
    padding = 3


    a_star = AStarPlanner(grid_size, robot_radius, padding)

    rx, ry, tpx, tpy, tvec = a_star.planning(sx, sy, gx, gy)


if __name__ == '__main__':
    main()



    