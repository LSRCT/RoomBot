import numpy as np
import pickle
import time
from PIL import Image
import matplotlib.pyplot as plt
from scipy import optimize

class RobotLocator:
    def __init__(self):
        self.img_room = Image.open("map4.png").convert("L")
        self.precalc_dist = self.load_precalc_dist()
        self.precalc_dist[1] = np.abs(self.precalc_dist[1].T[:3].T)
        self.loc_weights = np.zeros(len(self.precalc_dist[1]))
        self.pos_believe = np.array([0,0,0])
        print(f"weight before update {self.loc_weights[0]}")
    
    def load_precalc_dist(self):
        """ Load precalculated distances for a number of points from a pickle file"""
        pcdist = pickle.load(open("precalc_dist_Map4.p", "rb"))
        return pcdist

    def create_precalc_dist(self):
        """Precalculate distances and save as a pickle file """
        angles = list(range(0,360,10))
        positions, distances = self.calc_dist(1000, angles)
        positions = np.array(list(positions))
        distances = np.concatenate(distances, axis=0)
        pickle.dump([positions, distances, angles], open("precalc_dist_Map4.p", "wb"))

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


    def get_loc(self, sensor_data, return_list=1):
        """
        Get the closest position in the room to sensor data
        :param sensor_data: Measured sensor data
        :param return_list: If true returns while list, else just returns best 10 fitting positions
        """
        positions, distances, angles = self.precalc_dist
        print(distances[0])
        #print(f"shape possible positions: {np.shape(positions)}")
        #print(f"shape possible distances: {np.shape(distances)}")
        #print(distances[0], sensor_data[0], (distances-sensor_data[0])[0])
        dist_from_sim = np.linalg.norm((distances - sensor_data), axis=1)
        #print(np.shape(dist_from_sim))
        #dist_from_sim = np.array([np.linalg.norm(sensor_data[0][:3]-x[:3]) for x in distances])
        #min_ind = np.where(dist_from_sim == np.min(dist_from_sim))[0][0]
        min_pos_list = []
        sorted_ind = sorted(enumerate(dist_from_sim), key = lambda x: x[1])
        if return_list:
            return sorted_ind
        #print(np.shape(sorted_ind))
        for mind_numb in range(10):
            min_ind = np.where(dist_from_sim == np.min(dist_from_sim))[0][0]
            dist_from_sim[min_ind] = np.infty
            min_pos_list.append([positions[min_ind%len(positions)], distances[min_ind], angles[min_ind//len(positions)]])
        return min_pos_list
    
    def get_pos_from_ind(self, ind):
        """
        Return x,y coordinates of position with index ind
        :param ind: Index of position
        """
        state = np.concatenate((self.precalc_dist[0][ind%len(self.precalc_dist[0])], [self.precalc_dist[2][ind//len(self.precalc_dist[0])]]))
        return state
    
    def estimate_pos(self, sens_data):
        self.update_weights(self.get_loc(sens_data))
        self.pos_believe= self.get_pos_from_ind(np.argmax(self.loc_weights))
        return self.pos_believe

    def update_weights(self, min_pos_list):
        """
        Each location is a asigned a weitght based on the last measured sensor data, this function updates that list
        :param min_pos_list: List with possible positions ordered according to euclidean distance to measurement
        """
        #print(f"SHape min_pos_lsit: {np.shape(min_pos_list)}")
        min_pos_list = (np.array(min_pos_list).T[0].T).astype(int)
        #print(f"SHape min_pos_lsit: {np.shape(min_pos_list)}")
        print(f"Pos believe: {self.pos_believe[:2]}")
        new_weight = np.linspace(1,-1, len(min_pos_list)) 
        new_weight[:500] = new_weight[:500]**2
        dist_100toLast = np.linalg.norm(np.array([self.get_pos_from_ind(x)[:2] for x in min_pos_list[:100]])- self.pos_believe[:2], axis=1)
        #print(f"100 dist to last predict shape {np.shape(dist_100toLast)}, first: {dist_100toLast[0]}")
        min_pos_list[:100] = np.array(sorted(zip(dist_100toLast, min_pos_list), key=lambda x:x[0])).T[1].T
        new_est_sorted = np.array(sorted(zip(new_weight, min_pos_list), key=lambda x: x[1])).T[0].T
        self.loc_weights = 0.5* new_est_sorted + self.loc_weights
        #self.loc_weights = new_weight + self.loc_weights
        #print(f"weight after update {self.loc_weights[0]}")

    def plot_pos_dist(self, state):
        """
        Plot a position in the room
        :param state: [positions, distances, angle]
        """
        pos, dist_list, angle = state
        point = (pos[1], pos[0])
        img_rot = self.img_room.rotate(angle, expand=0, fillcolor=0, center=point)
        map_room = np.asarray(img_rot)
        map_room = (np.round(map_room/255, 1))
        plt.matshow(map_room)
        plt.scatter(point[0], point[1])
        plt.scatter(point[0]+dist_list[0], point[1])
        plt.scatter(point[0]+dist_list[2], point[1])
        plt.scatter(point[0], point[1]+dist_list[1])
        #plt.scatter(point[0], point[1]+dist_list[3])

if __name__ == "__main__":
    rb_loc = RobotLocator() 
    test_angle = 0
    positions, distances = rb_loc.calc_dist(100, [test_angle])
    t0 = time.time()
    res = rb_loc.get_loc(distances[0][4][:3], return_list=1)
    rb_loc.update_weights(res)
    print(res[2])
    #print(rb_loc.get_pos_from_ind(res[0][0]))
    print(f"Location took {time.time()-t0} seconds")
    rb_loc.create_precalc_dist()
    #rb_loc.plot_pos_dist(res[2])
    #rb_loc.plot_pos_dist([positions[4], distances[0][4][:3], test_angle])
    #plt.show()

    
