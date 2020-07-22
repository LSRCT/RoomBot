from RobotLocator import RobotLocator
from scipy.ndimage import uniform_filter
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
matplotlib.use("TkAgg")

class RBLocTester:
    def __init__(self, logfile):
        self.sdat_list = self.load_loc_data(logfile)
        self.rl = RobotLocator()
        self.map_room = self.rl.img_room
        self.prob_dist = self.rl.loc_weights[0]

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
            est.append(self.rl.estimate_pos([data[1]+5, data[0]+5, data[2]+5]))
            self.prob_dist = self.rl.loc_weights[0] + self.prob_dist
        est = np.array(est)
        self.est_pos = est
        return self.est_pos

    def plot_est_prob(self):
        #pos = np.array(list(self.rl.precalc_dist[0])*36)
        pos = np.concatenate(np.repeat([self.rl.precalc_dist[0]], 35, axis=0))
        print(set(list(self.rl.precalc_dist[2])))
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255
        print(np.sum(self.prob_dist))
        ##self.prob_dist = self.prob_dist[:len(pos)]
        print(np.shape(self.prob_dist), np.shape(pos))
        for coord, prob in zip(pos, self.prob_dist):
            map_room[coord[0]][coord[1]] += prob*255*2
        plt.matshow(map_room)
        
        plt.show()
        
        
    def plot_est_pos(self):
        plt.matshow(self.map_room)
        plt.scatter(self.est_pos[:,1], self.est_pos[:,0])
        plt.show()

dslice = slice(1900, 1950)
est = []
rbtest = RBLocTester("rb_datalog.csv") 
#rbtest.preprocess_locdata()
rbtest.estloc_datarange(dslice)
#urbtest.plot_est_pos()
rbtest.plot_est_prob()
