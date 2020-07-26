from RobotLocator import RobotLocator
from scipy.ndimage import uniform_filter1d
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
        self.sdat_list = np.array(self.sdat_list).T
        #self.sdat_list = uniform_filter1d(self.sdat_list, size=100, axis=0)
        self.sdat_list[0][5:] = winAvg(self.sdat_list[0], winWidth=20)[5:]
        self.sdat_list[1][5:] = winAvg(self.sdat_list[1], winWidth=20)[5:]
        self.sdat_list[2][5:] = winAvg(self.sdat_list[2], winWidth=20)[5:]
        print(len(self.sdat_list))
        self.sdat_list = np.array(self.sdat_list).T

    def estloc_datarange(self, datslice):
        est = []
        for data_numb, data in enumerate(self.sdat_list[datslice]):
            if data_numb%100 == 0:
                print(f"Estimated {data_numb} locations")
            for noisy_sens in self.apply_noise(data[1]+11, data[0]+11, data[2]+11):
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
        pos = self.rl.precalc_dist[0]
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255
        self.prob_dist = np.sum(self.prob_dist, axis=0)
        self.prob_dist = (self.prob_dist/np.max(self.prob_dist))*10
        print(np.sum(self.prob_dist))
        #self.prob_dist = self.prob_dist[:len(pos)]
        print(np.shape(self.prob_dist), np.shape(pos))
        bs= 2 # blobsize
        for coord, prob in zip(pos, self.prob_dist):
            map_room[coord[0]-bs:coord[0]+bs+1, coord[1]-bs:coord[1]+bs+1] += np.ones(((bs*2)+1,(bs*2)+1), dtype=np.uint8)* np.uint8(prob)
        plt.matshow(map_room)
        
        plt.show()

    def plot_raw(self, datslice):
        plt.plot(self.sdat_list[datslice])
        plt.show()
        
    def plot_est_pos(self):
        plt.matshow(self.map_room)
        #self.est_pos=self.rl.precalc_dist[0]
        plt.scatter(self.est_pos[:,1], self.est_pos[:,0])
        plt.show()


def winAvg(data, winWidth=0, winfunc=np.blackman, mode="same"):
    if not winWidth:
        if int(len(data)*(5/100))>0:
            winWidth = int(len(data)*(5/100))
        else: return data
    if not winWidth % 2:
        winWidth += 1
    kernel = winfunc(winWidth)/np.sum(winfunc(winWidth))
    data = np.convolve(data, kernel, mode=mode)
    return data

#dslice = slice(500, 3000)
dslice = slice(10, 90)
est = []
rbtest = RBLocTester("rb_datalog_bed.csv")
rbtest.preprocess_locdata()
rbtest.plot_raw(dslice)
rbtest.estloc_datarange(dslice)
rbtest.plot_est_pos()
rbtest.plot_est_prob()
