from RobotLocator import RobotLocator
from scipy.ndimage import uniform_filter
import matplotlib.pyplot as plt
import numpy as np

class RBLocTester:
    def __init__(self, logfile):
        self.sdat_list = self.load_loc_data(logfile)
        self.rl = RobotLocator()
        self.map_room = self.rl.img_room
        self.prob_dist = []

    def load_loc_data(self, logfile):
        sdat_list = []
        with open(logfile, "r") as f:
            for line in f:
                if line[0] != "t":
                    sdat_list.append([float(x) for x in line.split(";")[1:4]])
        return sdat_list

    def preprocess_locdata(self):
        n = len(self.sdat_list)
        self.sdat_list = uniform_filter(self.sdat_list, size=10)

    def estloc_datarange(self, datslice):
        est = []
        for data_numb, data in enumerate(self.sdat_list[datslice]):
            if data_numb%100 == 0:
                print(f"Estimated {data_numb} locations")
            for noisy_sens in self.apply_noise(data[1]+12, data[0]+12, data[2]+12):
                est.append(self.rl.estimate_pos(noisy_sens))
            #est.append(self.rl.estimate_pos([data[1]+12, data[0]+12, data[2]+12]))
            self.prob_dist.append(self.rl.loc_weights[0])
        est = np.array(est)
        self.est_pos = est
        return self.est_pos

    def apply_noise(self, x,y,z):
        return [[x,y,z]]
        new_coords = []
        noiserange = np.linspace(-0.5,0.5,3)
        print(noiserange)
        for noise_x in noiserange:
            for noise_y in noiserange:
                for noise_z in noiserange:
                    new_coords.append([x+noise_x, y+noise_y, z+noise_z])
        return new_coords

    def plot_est_prob(self):
        #pos = np.array(list(self.rl.precalc_dist[0])*36)
        #pos = np.concatenate(np.repeat([self.rl.precalc_dist[0]], 35, axis=0))
        pos = self.rl.precalc_dist[0]
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255
        self.prob_dist = np.sum(self.prob_dist[10:], axis=0)
        print(np.sum(self.prob_dist))
        ##self.prob_dist = self.prob_dist[:len(pos)]
        print(np.shape(self.prob_dist), np.shape(pos))
        for coord, prob in zip(pos, self.prob_dist):
            map_room[coord[0]][coord[1]] += prob*255*2
        plt.matshow(map_room)
        
        plt.show()
        
        
    def plot_est_pos(self):
        plt.matshow(self.map_room)
        #self.est_pos=self.rl.precalc_dist[0]
        plt.scatter(self.est_pos[:,1], self.est_pos[:,0])
        plt.show()

dslice = slice(2900, 3050)
est = []
rbtest = RBLocTester("rb_datalog.csv") 
#rbtest.preprocess_locdata()
rbtest.estloc_datarange(dslice)
rbtest.plot_est_pos()
rbtest.plot_est_prob()
