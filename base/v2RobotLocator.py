import numpy as np
from sklearn.preprocessing import minmax_scale
from scipy.spatial import distance
import pickle
import time
from PIL import Image
import matplotlib.pyplot as plt

class RobotLocator:
    def __init__(self):
        # load the map of the room
        self.img_room = Image.open("maps//map5.png").convert("L")

        # load simulated sensor values for map
        self.precalc_dist = self.load_precalc_dist()

        # originally 4 sensors where simulated, we only use 3
        self.precalc_dist[1] = np.abs(self.precalc_dist[1].T[:3].T)

        # initalize the grid as uniform distribution
        self.loc_weights = np.ones(len(self.precalc_dist[1]))/len(self.precalc_dist[1])

        # initial position believe TODO make a particle filter instead
        self.pos_believe = np.array([620,710,180])

        self.dist_list = 0
        self.pos_list = 0
        self.precalc_angles()
    

    def precalc_angles(self):
        """
        Add angles to precalc distances in the right format.
        """
        positions, distances = self.precalc_dist
        angles = positions.T[2].T
        angles = np.array([360-x if x >= 180 else x for x in angles])
        angles = angles*2
        self.precalc_dist[1] = np.concatenate((distances.T, np.array([angles]))).T


    def load_precalc_dist(self):
        """ Load precalculated distances for a number of points from a pickle file"""
        pcdist = pickle.load(open("precalc_distances//precalc_map5_5000.p", "rb"))
        return pcdist

    def calc_dist_list(self, sensor_data, max_dist = 200):
        """
        Calculate the euclidean distance of sensor data to simulated sensor data postions of the map
        :param sensor_data: Measured sensor data
        :param max_dist: Maximum distance for sensor, values bigger wont be considered
        """
        positions, distances = self.precalc_dist
        sensor_valid = []
        dist_comp = []
        for d, sval in enumerate(sensor_data[:3]):
            if sval < max_dist:
                sensor_valid.append(sval)
                dist_comp.append(distances.T[d])
        
        dist_comp = np.array(dist_comp)
        dist_comp = dist_comp.T
        self.dist_list = distance.cdist([sensor_valid], dist_comp, metric="euclidean")

    def calc_pos_list(self):
        """
        Calculate the distance of all possible points to the current estiamte.
        """
        positions, distances = self.precalc_dist
        positions = positions.copy().T
        positions[2] = np.abs(((positions[2] - self.pos_believe[2]+ 180)%360)-180)
        self.pos_calc = [*self.pos_believe[:2], 0]
        positions = positions.T
        self.pos_list = np.linalg.norm(positions - self.pos_calc, axis=1)

    def get_pos_from_ind(self, ind):
        """
        Return x,y coordinates of position with index ind
        :param ind: Index of position
        """
        state = self.precalc_dist[0][ind]
        return state
    
    def estimate_pos(self, sens_data, control_data):
        """
        Estime position of given sensor data and control data
        :param sens_data: [left, up, right]
        :param control_dat: [left, right]
        """
        # Sensor model
        self.calc_dist_list(np.array(sens_data))
        # Motion model
        self.update_position(control_data)
        self.calc_pos_list()
        # Localization
        self.update_weights()

        ind_max_prob = np.argmax(self.loc_weights)
        est_prob = np.max(self.loc_weights)
        self.pos_believe = self.get_pos_from_ind(ind_max_prob)
        return self.pos_believe

    def update_position(self, control_data):
        """
        Update the robots estimated position based on a shitty motion model and the robot controls
        """
        last_ins = control_data

        # robot velocity in 1/100 cm/s
        v_robot = 7.5
        dx = 0
        dy = 0
        phi = self.pos_believe[2]

        # forward
        if last_ins == [1024, 1024]:
            dx =  v_robot * np.sin(phi * np.pi/180)
            dy = v_robot * np.cos(phi * np.pi/180)
            dphi = 0
        # backwards
        elif last_ins == [-1024, -1024]:
            dx = -1 * v_robot * np.sin(phi * np.pi/180)
            dy = -1 * v_robot * np.cos(phi * np.pi/180)
            dphi = 0
        # spin
        elif last_ins in [[1024, -1024], [-1024, 1024]]:
            dx = 0
            dy = 0
            dphi = (last_ins[0] / 1024) * 20
        else:
            dx = last_ins[0] / 1024 * v_robot*np.sin(phi * np.pi / 180)
            dy = last_ins[1] / 1024 * v_robot*np.cos(phi * np.pi / 180)
            dphi = 0

        self.pos_believe[0] -= dy
        self.pos_believe[1] += dx
        self.pos_believe[2] += dphi
        self.pos_believe[2] = self.fix_phi_range(self.pos_believe[2])
        
    def fix_phi_range(self, phi):
        """
        Returns angle between 0 and 360
        """
        if phi > 360:
            phi = phi - 360
        elif phi < 0:
            phi = phi + 360
        return phi


    def update_weights(self):
        """
        Update the believe according to markov localization
        Bel(x_t) = alpha * P(s|x_t, map) * P(x_t|u_t, x_t-1)
        """
        # Gaussian
        def euclidean_to_x(d, mu=0, sigma=0.05):
            return (1/sigma*2.5)*np.e**(-0.5*(((d-mu)/sigma)**2))

        self.dist_list = self.dist_list/np.max(self.dist_list)

        #  this is P(s|x)
        dist_similarity = euclidean_to_x(self.dist_list)
        dist_similarity = dist_similarity/np.sum(dist_similarity)
        
        # Nothing is impossible
        self.sensor_weights = dist_similarity+1e-13

        # P(x_t|u_t, x_t-1)
        self.control_weights = euclidean_to_x(self.pos_list, mu=0, sigma=20)
        self.control_weights = self.control_weights/(np.sum(self.control_weights))

        # Bel(x)
        self.loc_weights = self.sensor_weights * self.control_weights

        # alpha to make it integrate to 1, this is the alpha
        self.loc_weights = self.loc_weights/(np.sum(self.loc_weights))

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

if __name__ == "__main__":
    rb_loc = RobotLocator() 
    test_angle = 0
    test_ind = 1000
    test_sensordat = rb_loc.precalc_dist[1][test_ind][:3]
    test_sensordat = np.concatenate((test_sensordat, [4.5]))
    test_groundtruth = rb_loc.precalc_dist[0][test_ind]
    t0 = time.time()
    print(f"Test sensordata: {test_sensordat}")
    res = rb_loc.estimate_pos(test_sensordat, [0,0])
    print(f"Groundtruth: {test_groundtruth} Estimate: {res}")
    print(f"Location took {time.time()-t0} seconds")
    rb_loc.plot_pos_dist([test_groundtruth, test_sensordat, 0])
    plt.show()

