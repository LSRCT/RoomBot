import numpy as np
from sklearn.preprocessing import minmax_scale
from scipy.spatial import distance
import pickle
import time
from PIL import Image
import matplotlib.pyplot as plt

class RobotLocator:
    def __init__(self):
        self.img_room = Image.open("maps//map5.png").convert("L")
        self.precalc_dist = self.load_precalc_dist()
        print(self.precalc_dist[0][-1])
        self.precalc_dist[1] = np.abs(self.precalc_dist[1].T[:3].T)
        self.loc_weights = np.ones(len(self.precalc_dist[1]))/len(self.precalc_dist[1])
        self.pos_believe = np.array([920,735,0])
        self.dist_list = 0
    
    def load_precalc_dist(self):
        """ Load precalculated distances for a number of points from a pickle file"""
        pcdist = pickle.load(open("precalc_distances//precalc_map5_5000.p", "rb"))
        return pcdist

    def calc_dist_list(self, sensor_data, return_list=1):
        """
        Calculate the euclidean distance of sensor data to simulated sensor data postions
        :param sensor_data: Measured sensor data
        :param return_list: If true returns while list, else just returns best 10 fitting positions
        """
        positions, distances = self.precalc_dist
        angles = positions.T[2].T-sensor_data[3]
        angles = np.abs((angles +180) % 360 -180)/2
        angles = np.sin(np.deg2rad(angles))
        angles = ((angles*2)**3)*100
        distances = np.concatenate((distances.T, np.array([angles]))).T
        #print(np.shape(distances))
        sensor_data[3] = 0#np.sin(sensor_data[3]/2)*10
        #print(sensor_data[3])
        print(angles[0])
        self.dist_list = distance.cdist([sensor_data], distances, metric="euclidean")

    def get_pos_from_ind(self, ind):
        """
        Return x,y coordinates of position with index ind
        :param ind: Index of position
        """
        #state = np.concatenate((self.precalc_dist[0][ind%len(self.precalc_dist[0])], [self.precalc_dist[2][ind//len(self.precalc_dist[0])]]))
        state = self.precalc_dist[0][ind]
        return state
    
    def estimate_pos(self, sens_data):
        """
        Estime position of given sensor data
        :param sens_data: [left, up, right]
        """
        self.calc_dist_list(np.array(sens_data))
        self.update_weights()
        ind_max_prob = np.argmax(self.loc_weights)
        est_prob = np.max(self.loc_weights)
        self.pos_believe= self.get_pos_from_ind(np.argmax(self.loc_weights))
        return self.pos_believe

    def update_weights(self):
        # Bel(x) = alpha*P(s|x)*Bel(x)
        #  this is P(s|x)
        def euclidean_to_x(d):
            mu = -7
            sigma =2 
            return (1/sigma*2.5)*np.e**(-0.5*(((d-mu)/sigma)**2))
        #dist_similarity = 1/(1+euclidean_to_x(self.dist_list/np.max(self.dist_list)))
        print(len(self.dist_list[0]))
        import matplotlib.pyplot as plt
        self.dist_list = self.dist_list/np.max(self.dist_list)
        #plt.plot(sorted(self.dist_list[0]))
        dist_similarity = euclidean_to_x(self.dist_list)
        dist_similarity = dist_similarity/np.max(dist_similarity)
        #plt.plot(sorted(dist_similarity[0], reverse=1))
        
        #plt.show()
        # update believe
        self.loc_weights = dist_similarity*self.loc_weights
        print(np.max(self.loc_weights))
        # alpha to make it integrate to 1
        self.loc_weights = self.loc_weights/(np.sum(self.loc_weights))
        print(np.max(self.loc_weights))

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
        plt.scatter(point[0]-dist_list[0], point[1])
        plt.scatter(point[0]+dist_list[2], point[1])
        plt.scatter(point[0], point[1]-dist_list[1])
        #plt.scatter(point[0], point[1]+dist_list[3])

if __name__ == "__main__":
    rb_loc = RobotLocator() 
    test_angle = 0
    test_ind = 10
    test_sensordat = rb_loc.precalc_dist[1][test_ind]
    test_sensordat = np.concatenate((test_sensordat, [4.5]))
    test_groundtruth = rb_loc.precalc_dist[0][test_ind]
    t0 = time.time()
    res = rb_loc.estimate_pos(test_sensordat)
    print(f"Groundtruth: {test_groundtruth} Estimate: {res}")
    print(f"Location took {time.time()-t0} seconds")
    rb_loc.plot_pos_dist([test_groundtruth, test_sensordat, 0])
    plt.show()

