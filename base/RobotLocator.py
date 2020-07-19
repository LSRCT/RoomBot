import numpy as np
import pickle
import time
from PIL import Image
import matplotlib.pyplot as plt

class RobotLocator:
    def __init__(self):
        self.img_room = Image.open("map5.png").convert("L")
        self.precalc_dist = self.load_precalc_dist()
        self.precalc_dist[1] = np.abs(self.precalc_dist[1].T[:3].T)
        self.loc_weights = np.zeros(len(self.precalc_dist[1]))
        self.pos_believe = np.array([0,0,0])
        self.dist_list = 0
    
    def load_precalc_dist(self):
        """ Load precalculated distances for a number of points from a pickle file"""
        pcdist = pickle.load(open("precalc_dist_Map5.p", "rb"))
        return pcdist



    def calc_dist_list(self, sensor_data, return_list=1):
        """
        Calculate the euclidean distance of sensor data to simulated sensor data postions
        :param sensor_data: Measured sensor data
        :param return_list: If true returns while list, else just returns best 10 fitting positions
        """
        positions, distances, angles = self.precalc_dist
        dist_from_sim = np.linalg.norm((distances - sensor_data), axis=1)
        self.dist_list = dist_from_sim
    
    def get_pos_from_ind(self, ind):
        """
        Return x,y coordinates of position with index ind
        :param ind: Index of position
        """
        state = np.concatenate((self.precalc_dist[0][ind%len(self.precalc_dist[0])], [self.precalc_dist[2][ind//len(self.precalc_dist[0])]]))
        return state
    
    def estimate_pos(self, sens_data):
        self.calc_dist_list(sens_data)
        self.update_weights()
        self.pos_believe= self.get_pos_from_ind(np.argmax(self.loc_weights))
        return self.pos_believe

    def update_weights(self):
        """
        Each location is a asigned a weitght based on the last measured sensor data, this function updates that list
        :param min_pos_list: List with possible positions ordered according to euclidean distance to measurement
        """
        #for mind_numb in range(10):
        #    min_ind = np.where(dist_from_sim == np.min(dist_from_sim))[0][0]
        #    dist_from_sim[min_ind] = np.infty
        #    min_pos_list.append([positions[min_ind%len(positions)], distances[min_ind], angles[min_ind//len(positions)]])

        # Sort the possible positions by their similarity to measured sensor data
        sorted_dist_list = sorted(enumerate(self.dist_list), key = lambda x: x[1])
        sorted_ind_list = (np.array(sorted_dist_list).T[0].T).astype(int)

        new_weight = np.linspace(1,-1, len(sorted_ind_list)) 
        new_weight[:500] = new_weight[:500]**2

        # Sort the top100 sensor dist fits by their distance to the last estimated position
        dist_100toLast = np.linalg.norm(np.array([self.get_pos_from_ind(x)[:2] for x in sorted_ind_list[:100]])- self.pos_believe[:2], axis=1)
        sorted_ind_list[:100] = np.array(sorted(zip(dist_100toLast, sorted_ind_list), key=lambda x:x[0])).T[1].T
        
        # Give each position a new weight based on its positin in the sorted_ind_list
        new_est_sorted = np.array(sorted(zip(new_weight, sorted_ind_list), key=lambda x: x[1])).T[0].T
        self.loc_weights = 0.5* new_est_sorted + self.loc_weights

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
    test_sensordat = rb_loc.precalc_dist[1][10]
    test_groundtruth = rb_loc.precalc_dist[0][10]
    t0 = time.time()
    res = rb_loc.estimate_pos(test_sensordat)
    print(f"Groundtruth: {test_groundtruth} Estimate: {res}")
    print(f"Location took {time.time()-t0} seconds")
