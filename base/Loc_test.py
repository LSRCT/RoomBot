from RobotLocator import RobotLocator
from scipy.ndimage import uniform_filter
import matplotlib.pyplot as plt
import numpy as np

class RBLocTester:
    def __init__(self, logfile):
        self.sdat_list = self.load_loc_data(logfile)
        self.rl = RobotLocator()
        self.map_room = self.rl.img_room

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
            est.append(self.rl.estimate_pos([data[1]+12, data[0]+12, data[2]+12]))
        est = np.array(est)
        self.est_pos = est
        return self.est_pos

    def plot_est_pos(self):
        self.est_pos = self.est_pos[-100:]
        plt.matshow(self.map_room)
        plt.scatter(self.est_pos[:,1], self.est_pos[:,0])
        plt.show()

dslice = slice(0, 500)
est = []
rbtest = RBLocTester("rb_datalog.csv") 
#rbtest.preprocess_locdata()
rbtest.estloc_datarange(dslice)
rbtest.plot_est_pos()
