from RobotLocator import RobotLocator
import cv2
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
        """
        Load recorded sensor data from csv file
        """
        sdat_list = []
        with open(logfile, "r") as f:
            for line in f:
                if line[0] != "t":
                    sdat_list.append([float(x) for x in line.split(";")[1:5]])
        return sdat_list

    def preprocess_locdata(self):
        """
        Apply moving average to compensate for outliers
        """
        n = len(self.sdat_list)
        self.sdat_list = np.array(self.sdat_list).T
        self.sdat_list[0][5:] = winAvg(self.sdat_list[0], winWidth=10)[5:]
        self.sdat_list[1][5:] = winAvg(self.sdat_list[1], winWidth=10)[5:]
        self.sdat_list[2][5:] = winAvg(self.sdat_list[2], winWidth=10)[5:]
        self.sdat_list = np.array(self.sdat_list).T

    def estloc_datarange(self, datslice):
        """
        Estimate probability maps for all datapoints
        :param dataslice: (start, stop) data range to estimate
        """
        est = []
        rshap = (len(set(self.rl.precalc_dist[0].T[2])), -1)
        for data_numb, data in enumerate(self.sdat_list[datslice]):
            if data_numb%100 == 0:
                print(f"Estimated {data_numb} locations")
            for noisy_sens in self.apply_noise(data[1]+11, data[0]+11, data[2]+11, data[3]):
                est.append(self.rl.estimate_pos(noisy_sens))
            l_w = np.sum(np.reshape(self.rl.loc_weights, rshap), axis=0)
            self.prob_dist.append(l_w)
        est = np.array(est)
        self.est_pos = est
        return self.est_pos

    def apply_noise(self, x,y,z, phi):
        """
        Augment the data by applying a fixed amount of offset on every axis
        """
        return [[x,y,z, phi]]
        new_coords = []
        noiserange = np.linspace(-0.5,0.5,3)
        for noise_x in noiserange:
            for noise_y in noiserange:
                for noise_z in noiserange:
                    new_coords.append([x+noise_x, y+noise_y, z+noise_z])
        return new_coords

    def est_prob_video(self,datslice):
        """
        Show the probabiliy map for a dataslice as a video
        """
        # load simulated distances and room map
        pos = self.rl.precalc_dist[0]
        pos = np.reshape(pos, (len(set(pos.T[2])), -1, 3))[0]
        rshap = (len(set(self.rl.precalc_dist[0].T[2])), -1)
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255

        # estimate position for all selected data points
        for data_numb, data in enumerate(self.sdat_list[datslice]):
            self.rl.estimate_pos([data[1]+11, data[0]+11, data[2]+11, data[3]])
            l_w = np.sum(np.reshape(self.rl.loc_weights, rshap), axis=0)
            prob = (l_w/np.max(l_w))*255
            bs= 2 # blobsize
            for coord, p in zip(pos, prob):
                map_room[coord[0]-bs:coord[0]+bs+1, coord[1]-bs:coord[1]+bs+1] = np.ones(((bs*2)+1,(bs*2)+1), dtype=np.uint8)* np.uint8(p)
            map_room_crop = map_room[550:1000, 650:1250]
            cv2.imshow('Frame', map_room_crop)
            if cv2.waitKey(2) & 0xFF == ord("q"):
                break
        cv2.destroyAllWindows()

    def plot_est_prob(self, frame, ax=None):
        """
        Plot the probability map for a single datapoint
        :param frame: index of datapoint to plot
        :param ax: matplotlib axis to plot to, if none show image
        """
        # load simulated distances and room map
        pos = self.rl.precalc_dist[0]
        pos = np.reshape(pos, (len(set(pos.T[2])), -1, 3))[0]
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)

        # estimate position from data
        data = self.sdat_list[frame]
        self.rl.estimate_pos([data[1]+11, data[0]+11, data[2]+11, data[3]])
        rshap = (len(set(self.rl.precalc_dist[0].T[2])), -1)
        l_w = np.sum(np.reshape(self.rl.loc_weights, rshap), axis=0)+0.001
        prob = (l_w/np.max(l_w))*255

        # Plot the probability map
        bs= 2 # blobsize
        for coord, p in zip(pos, prob):
            map_room[coord[0]-bs:coord[0]+bs+1, coord[1]-bs:coord[1]+bs+1] = np.ones(((bs*2)+1,(bs*2)+1), dtype=np.uint8)* np.uint8(p)
        map_room_crop = map_room[550:1000, 650:1250]
        if ax == None:
            plt.matshow(map_room_crop)
            plt.show()
        else:
            ax.matshow(map_room_crop)

    
    def plot_prob_timeline(self, plot_ind):
        """
        Plot multiple probability maps in a row to show time progression
        :param plot_ind: list containing data indicies to plot
        """
        fig, ax_list = plt.subplots(1, len(plot_ind), sharey=True)
        for ind_numb, (ind, ax) in enumerate(zip(plot_ind, ax_list)):
            ax.set_title(f"T = {ind_numb*2}s")
            self.plot_est_prob(ind, ax=ax)
            ax.axis('off')
        plt.tight_layout()
        plt.show()


    def plot_raw(self, datslice):
        """
        Plot the raw recorded data
        """
        plt.plot(self.sdat_list[datslice])
        plt.show()
        
    def plot_est_pos(self):
        plt.matshow(self.map_room)
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


rbtest = RBLocTester("rb_datalog_phi.csv")
rbtest.preprocess_locdata()
#rbtest.est_prob_video(dslice)
#rbtest.plot_raw(dslice)
#rbtest.estloc_datarange(dslice)
rbtest.plot_prob_timeline([220, 240, 260, 280])
