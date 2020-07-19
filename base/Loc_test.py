from RobotLocator import RobotLocator
import matplotlib.pyplot as plt
import numpy as np

class RBLocTester:
    def __init__(self, logfile):
        self.sdat_list = self.load_loc_data(logfile)
        self.rl = RobotLocator()
        self.map_room = self.rl.img_room
        self.datarange = slice(0,-1)

    def load_loc_data(self, logfile):
        sdat_list = []
        with open(logfile, "r") as f:
            for line in f:
                if line[0] != "t":
                    sdat_list.append([float(x) for x in line.split(";")[1:4]])
        return sdat_list

    def estloc_datarange(self, datslice):
        est = []
        for data_numb, data in enumerate(self.sdat_list[datslice]):
            if data_numb%100 == 0:
                print(f"Estimated {data_numb} locations")
            est.append(np.concatenate((self.rl.estimate_pos([data[1]+12, data[0]+12, data[2]+12]), [data_numb])))
            print(est[-1])
        est = np.array(est)
        self.datarange = datslice
        self.est_pos = est
        return self.est_pos

    def plot_est_pos(self):
        plt.matshow(self.map_room)
        plt.scatter(self.est_pos[:,1], self.est_pos[:,0])
        plt.show()

dslice = slice(600, 900)
est = []
rbtest = RBLocTester("rb_datalog4.csv") 
rbtest.estloc_datarange(dslice)
rbtest.plot_est_pos()
