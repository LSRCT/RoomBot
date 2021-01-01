from v2RobotLocator_PF import RobotLocator
import cv2
from scipy.ndimage import uniform_filter1d
import matplotlib.pyplot as plt
import numpy as np

class RBLocTester:
    def __init__(self, logfile):
        self.sdat_list, self.cdat_list = self.load_loc_data(logfile)
        self.rl = RobotLocator()
        self.map_room = self.rl.img_room
        self.prob_dist = []

    def load_loc_data(self, logfile):
        """
        Load recorded sensor data from csv file
        """
        sdat_list = []
        cdat_list = []
        with open(logfile, "r") as f:
            for line in f:
                if line[0] != "t":
                    ls = [float(x) for x in line.split(";")[:7]]
                    sdat_list.append(ls[1:5])
                    cdat_list.append(ls[5:7])
        return sdat_list, cdat_list

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

    def est_prob_video(self, datslice, save=0, show=1):
        """
        Show the probabiliy map for a dataslice as a video
        """
        # load simulated distances and room map
        pos = self.rl.precalc_dist[0]
        pos = np.reshape(pos, (len(set(pos.T[2])), -1, 3))[0]
        rshap = (len(set(self.rl.precalc_dist[0].T[2])), -1)
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255
        if save:
            save_name = "rb_mov.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(save_name, fourcc, 10.0,(600, 450), isColor=False)

        # estimate position for all selected data points
        for data_numb, (data, control) in enumerate(zip(self.sdat_list[datslice], self.cdat_list[datslice])):
            self.rl.estimate_pos([data[1], data[0], data[2], data[3]], control)
            l_w = np.sum(np.reshape(self.rl.loc_weights, rshap), axis=0)
            prob = (l_w/np.max(l_w))*255
            bs= 2 # blobsize
            for coord, p in zip(pos, prob):
                map_room[coord[0]-bs:coord[0]+bs+1, coord[1]-bs:coord[1]+bs+1] = np.ones(((bs*2)+1,(bs*2)+1), dtype=np.uint8)* np.uint8(p)
            map_room_crop = map_room[550:1000, 650:1250]
            print(np.shape(map_room_crop))
            if save:
                out.write(map_room_crop)
            if show:
                cv2.imshow('Frame', map_room_crop)
                if cv2.waitKey(2) & 0xFF == ord("q"):
                    break
        if save:
            out.release()
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
        control = self.cdat_list[frame]
        self.rl.estimate_pos([data[1], data[0], data[2], data[3]], control)
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

    def est_particle_video(self, datslice, save=0, show=1):
        """
        Show the particles for a dataslice as a video
        """
        # load simulated distances and room map
        pos = self.rl.precalc_dist[0]
        pos = np.reshape(pos, (len(set(pos.T[2])), -1, 3))[0]
        rshap = (len(set(self.rl.precalc_dist[0].T[2])), -1)
        map_room = np.asarray(self.map_room)
        map_room = np.abs(np.round(map_room, 1)-255)*255
        if save:
            save_name = "rb_mov.avi"
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(save_name, fourcc, 10.0,(600, 450), isColor=False)

        # estimate position for all selected data points
        for data_numb, (data, control) in enumerate(zip(self.sdat_list[datslice], self.cdat_list[datslice])):
            map_room = np.asarray(self.map_room)
            map_room = np.abs(np.round(map_room, 1)-255)*255
            self.rl.estimate_pos([data[1], data[0], data[2], data[3]], control)
            particles = self.rl.particles
            bs= 2 # blobsize
            for coord in particles:
                map_room[coord[0]-bs:coord[0]+bs+1, coord[1]-bs:coord[1]+bs+1] = np.ones(((bs*2)+1,(bs*2)+1), dtype=np.uint8)* np.uint8(255)
            map_room_crop = map_room[550:1000, 650:1250]
            if save:
                out.write(map_room_crop)
            if show:
                cv2.imshow('Frame', map_room_crop)
                if cv2.waitKey(2) & 0xFF == ord("q"):
                    break
        if save:
            out.release()
        cv2.destroyAllWindows()
    
    
    def plot_position_history(self):
        plt.scatter(self.cdat_list.T[0].T, self.cdat_list.T[1].T)
        plt.show()

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


rbtest = RBLocTester("rb_data.csv")
dslice = slice(0, 390)
rbtest.est_particle_video(dslice, save=1, show=1)
#rbtest.plot_raw(dslice)
#rbtest.plot_prob_timeline([220, 240, 260, 280])
