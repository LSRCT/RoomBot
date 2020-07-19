import sys
import numpy as np
import pickle
import time
from PIL import Image
import matplotlib.pyplot as plt
from scipy import optimize

class RBMapCalculator:
    def __init__(self,RoomPng):
        self.img_room = Image.open(RoomPng).convert("L")
        self.img_room_name = RoomPng

    def create_precalc_dist(self, filename=None):
        """
        Precalculate distances and save as a pickle file 
        """
        print(f"Calculating positions with corresponding distances for {self.img_room_name}")
        print(f"This may take a while ...")
        if filename==None:
            fname = "precalc_dist_"+self.img_room_name[:-2]
        else:
            fname = filename
        
        angles = list(range(0,360,10))
        positions, distances = self.calc_dist(1000, angles)
        positions = np.array(list(positions))
        distances = np.concatenate(distances, axis=0)
        pickle.dump([positions, distances, angles], open(fname, "wb"))
        
        print(f"Done. saving as {fname}")

    def calc_dist(self, grid_points, angles):
        """
        Calculate the distances for specified amount of about evenely spaces grid points in self.img_room
        :param grid_points: Number of points the grid of the room should have
        :param angles: Angles for each points
        """
        valid_positions = self.get_valid_positions()
        valid_positions = valid_positions[::((len(valid_positions)-1)//grid_points)]
        distances_rot = []
        for phi in angles:
            distances_rot.append(self.calc_all_rot(valid_positions, phi))
        return valid_positions, distances_rot

    def get_valid_positions(self):
        """
        Return the pixels of self.img_room inside the room
        """
        map_room = np.asarray(self.img_room)
        map_room = (np.round(map_room/255, 1))
        v_positions = np.array(np.where(map_room == 1)).T
        return v_positions

    def calc_all_rot(self, locations, angle):
        """
        Calculate distances for specified locations for all specified angle
        :param locations: Points to calculate distances for
        :param angle: Angle to use for calculation
        """
        distances = []
        for pos in locations:
            dist = self.calc_rot_single([pos[0], pos[1], angle])
            distances.append(dist)
        return np.array(distances)


    def get_valid_inters(self, plist, pos_x):
        """
        Get valid intersections out of all intersections with walls
        :param plist: Intersection points
        :param pos_x: Original coordinate of the point for 1 axis
        """
        # rotation causes aliasing
        plist = np.array(list(set(np.round(plist/2)*2)))  # TODO this is bad
        points_valid = [plist[0], plist[1]]
        # so we get the closest 2
        for point in plist:
            if pos_x > point and (point > points_valid[0] or points_valid[0] > pos_x):
                points_valid[0] = point
            if pos_x < point and (point < points_valid[1] or points_valid[1] < pos_x):
                points_valid[1] = point
        return points_valid

    def calc_rot_single(self, state):
        """
        Calculate distances of a rotated points
        :param state: [x_coordinate, ycoordinate, angle]
        """
        angle = state[2]
        point = (state[1], state[0])
        img_rot = self.img_room.rotate(angle, expand=0, fillcolor=0, center=point)
        map_room = np.asarray(img_rot)
        map_room = map_room / 255
        dist = self.calc_dist_single(map_room, point)
        return dist

    def calc_dist_single(self, map_room, pos):
        """
        Calculate the dsitances of a positions given a room map
        :param map_room: Map to calculate the distances to walls in 
        :param pos: Position to calculate distances from 
        """
        x_points = np.where(map_room[pos[1]] == 0)[0]
        x_points = self.get_valid_inters(x_points, pos[0])
        x_dist = [[x_points[0], pos[1]], [x_points[1], pos[1]]]
        y_points = np.where(map_room[:,pos[0]] == 0)[0]
        y_points = self.get_valid_inters(y_points, pos[1])
        y_dist = [[pos[0], y_points[0]], [pos[0], y_points[1]]]
        dist = [x_points[0]-pos[0], y_points[0]-pos[1], x_points[1]-pos[0], y_points[1]-pos[1]]
        return np.array(dist)

if __name__ == "__main__":
    if len(sys.argv) == 2:
        map_name = sys.argv[1] 
        print(map_name)
    else:
        print(f"Syntax is RBMapCalculator.py map.png")
        sys.exit()
    rbm = RBMapCalculator(map_name)
    rbm.create_precalc_dist()

    
