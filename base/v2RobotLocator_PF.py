import numpy as np
import pprint
import random
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
        self.calc_pos_list()
        self.loc_weights = euclidean_to_x(self.pos_list, sigma=50)
        #self.loc_weights = np.ones(self.loc_weights.shape)
        self.loc_weights = self.loc_weights/np.sum(self.loc_weights)
        self.particles = []
        n_particles = 1000
        for n in range(n_particles):
            p_ind = self.sample_from_bel()
            self.particles.append(self.get_pos_from_ind(p_ind))
        self.particles = np.array(self.particles)
        self.weights = np.zeros((self.particles.shape[0]))
        self.dist_list = 0
        self.pos_list = 0
        self.precalc_angles()
        self.precalc_loc_dist()
    
    def precalc_loc_dist(self):
        """
        Save simluated sensor data as grid for easy access
        """
        # x 585 y 695 phi 0
        # x 990 y 1200 phi 355
        self.occ_grid = np.ones(((990-585)//5+1, (1200-695)//5+1, (355-0)//5+1, 3))*50000
        positions, distances = self.precalc_dist
        for p, d in zip(*self.precalc_dist):
            self.occ_grid[(p[0]-585)//5][(p[1]-695)//5][(p[2]//5)] = d[:3]


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

    def calc_dist_list(self, sensor_data, max_dist = 300):
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
        
        if len(dist_comp) == 0:
            sensor_valid = [0]
            dist_comp.append(np.ones(distances.T[0].shape))
        dist_comp = np.array(dist_comp)
        dist_comp = dist_comp.T
        self.dist_list = distance.cdist([sensor_valid], dist_comp, metric="euclidean")

    def calc_weight_index(self, sensor_data, pos_index, max_dist=250):
        sensor_valid = []
        dist_comp = []
        dist = self.precalc_dist[1][pos_index]
        for d, sval in zip(dist, sensor_data[:3]):
            if sval < max_dist:
                sensor_valid.append(sval)
                dist_comp.append(d)
        ret = np.linalg.norm(np.array(sensor_valid)-np.array(dist_comp))
        return ret

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
        state = self.precalc_dist[0][ind].copy()
        return state

    def estimate_pos(self, sens_data, control_data):
        """
        Estime position of given sensor data and control data
        :param sens_data: [left, up, right]
        :param control_dat: [left, right]
        """
        # Localization
        self.update_weights(control_data, sens_data)

        #ind_max_prob = np.argmax(self.loc_weights)
        #est_prob = np.max(self.loc_weights)
        #self.pos_believe = self.get_pos_from_ind(ind_max_prob)
        #return self.pos_believe


    def resample_particles(self, partciles, weights):
        """
        Resample particles with replacement from old particle distribution
        :param particles: Old particles
        :param weights: weights for old partiles
        :return: Resampled particle list
        """
        new_particles = []
        ind_list = list(range(len(self.particles)))
        for n in ind_list:
            new_particles.append(self.particles[np.random.choice(ind_list, p=weights)])
        return new_particles

    def get_sensor_from_pos(self, pos):
        """
        Get simulated sensor data for a certain position
        :param pos: xyphi position
        :return: Left top right sensor values
        """
        # x 585 y 695 phi 0
        # x 990 y 1200 phi 355
        if 585 <= pos[0] < 990 and 695 <= pos[1] < 1200:
            gc1 = (int(pos[0])-585)//5
            gc2 = (int(pos[1])-695)//5
            gc3 = (int(pos[2]))//5
            simssense = self.occ_grid[gc1][gc2][gc3]
        else:
            simssense = np.array([50000, 50000, 50000])
        return simssense
    
    def get_weights_sensor(self, sense, sim, max_dist=300):
        """
        Get the weights for a particle. Similarity measure of sensor data and simulated sensor data.
        """
        diffn = [se-si+np.random.normal(scale=0.5) for se,si in zip(sense, sim) if se < max_dist]
        w = np.linalg.norm(diffn)
        return w

    def update_weights(self, control_data, sens_data):
        """
        Update the believe according to markov localization
        Bel(x_t) = alpha * P(s|x_t, map) * P(x_t|u_t, x_t-1)
        """

        # Motion model
        #self.calc_dist_list(np.array(sens_data))
        for pnum, pos in enumerate(self.particles):
            # Motion model 
            pos_new = self.update_position(control_data, pos)
            
            ss = self.get_sensor_from_pos(pos_new)
            
            # weights from sensor model
            w = self.get_weights_sensor(sens_data[:3], ss)
            self.weights[pnum] = w
            self.particles[pnum] = pos_new

        self.weights = self.weights / np.max(self.weights)
        self.weights = euclidean_to_x(self.weights, sigma=0.5)
        self.weights = self.weights / np.sum(self.weights)
        self.particles = self.resample_particles(self.particles, self.weights)

    def sample_from_bel(self):
        cdf = np.cumsum(self.loc_weights)
        # SIMULATIONSLEMMA
        sample_ind = np.argmin(np.abs(cdf-random.random()))
        sample = self.get_pos_from_ind(sample_ind)
        return sample_ind


    def update_position(self, control_data, start_pos):
        """
        Update the robots estimated position based on a shitty motion model and the robot controls
        """
        last_ins = control_data
        new_pos = [0, 0, 0]

        # robot velocity in 1/100 cm/s
        v_robot = 7.5 + np.random.normal(scale=2)
        #v_robot = 100
        dx = 0
        dy = 0
        phi = start_pos[2]+np.random.normal(scale=5)

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
            dphi = (last_ins[0] / 1024) * 20 +  np.random.normal(loc=0, scale=5)
        else:
            dx = last_ins[0] / 1024 * v_robot*np.sin(phi * np.pi / 180)
            dy = last_ins[1] / 1024 * v_robot*np.cos(phi * np.pi / 180)
            dphi = 0

        new_pos[0] = start_pos[0] - dy# + np.random.normal(scale=0.1)
        new_pos[1] = start_pos[1] + dx# + np.random.normal(scale=0.1)
        new_pos[2] = start_pos[2] + dphi# + np.random.normal(scale=1)
        #print(np.random.normal(scale=1))
        new_pos[2] = self.fix_phi_range(new_pos[2])
        new_pos = list(map(int, new_pos))
        return new_pos
        
    def fix_phi_range(self, phi):
        """
        Returns angle between 0 and 360
        """
        if phi >= 360:
            phi = phi - 360
        elif phi < 0:
            phi = phi + 360
        return phi


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

# Gaussian
def euclidean_to_x(d, mu=0, sigma=5):
    a = (d-mu)/sigma
    return (1/sigma*2.5)*np.e**(-0.5*(a*a))

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

