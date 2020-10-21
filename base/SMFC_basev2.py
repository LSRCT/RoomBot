import time
import os, sys
import numpy as np
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import getch
import pygame
import pygame.locals
import random


class SMFCBase(mqtt.Client):
    def __init__(self, ip, port=1883, manual=0, log_data=0):
        """
        Init the server with port and root directory
        :param ip: IP of the mqtt broker
        :param port: Port for mqtt broker
        :param manual: Manual controll on/off
        """
        super().__init__(client_id="SMFC_base")
        self.control_mode = manual 

        # connect to mqtt broker
        self.connect("192.168.178.27", port=port) 
        self.loop_start()
        self.fwd = 0

        self.roombot = RoomBot(log_data=log_data, disable_planning=manual)

        if manual:
            pygame.init()
            BLACK = (0,0,0)
            WHITE = (255,255,255)
            HEIGHT = 1024
            WIDTH = 1280
            self.display = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
            self.display.fill(BLACK)
            pygame.draw.rect(self.display, WHITE,(200,150,100,50))

    def __del__(self):
        print("Server stopped")


    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        self.subscribe("RR/sensors")
        self.subscribe("RR/magnetometer")

    def on_message(self, client, userdata, msg):
        if msg.topic == "RR/sensors": 
            self.on_message_sensors(msg)
        elif msg.topic == "RR/magnetometer":
            self.on_message_mag(msg)

    def on_message_sensors(self,msg):
        """
        Callback when distance sensor message is recieved.
        Decodes the message and passes it to RoomBot
        """
        data_rcv = msg.payload.hex()
        dist1 = ((int(data_rcv[0:4], 16))/10)
        dist2 = ((int(data_rcv[4:8], 16))/10)
        dist3 = ((int(data_rcv[8:12], 16))/10)
        self.roombot.handle_dist_data(dist1, dist2, dist3)

    def on_message_mag(self,msg):
        """
        Callback when magnetometer msg is recieved.
        Decodes the message and passes it to roombot
        """
        data_rcv = msg.payload.hex()
        magX = (int(data_rcv[0:4], 16)-200)/10
        magY = (int(data_rcv[4:8], 16)-200)/10
        self.roombot.handle_mag_data(magX, magY)

    def serve(self):
        """
        Start the server and serve forever
        """
        print("Server stared")
        ins = [0,0]
        next_ins = [0,0]
        t0 = time.time()
        robot_clock = 0.1
        while 1:
            # check for key input in manual mode
            if self.control_mode:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN:
                        key_in = pygame.key.name(event.key)
                        key_ins = self.handle_keyin(key_in)
                        self.roombot.ins_input = key_ins

            if time.time()-t0 > robot_clock:
                self.roombot.execute_timestep()
                next_ins = self.roombot.get_instruction()
                self.publish("RR/driveIns", self.conv_ar_to_msg(next_ins))
                t0 = time.time()
            time.sleep(0.001)

    def handle_keyin(self, key):
        """
        Determine instruction from key input for manual controll
        How to:
        w - forward
        s - backwards
        a - left
        d - right
        x - stop
        q - turn left in place
        e - turn right in place
        """
        print(f"handling {key}")
        r = 1024
        l = 1024
        fwdL = self.fwd
        fwdR = self.fwd
        if key == "w":
            self.fwd = 1
            fwdL = 1
            fwdR = 1
        elif key == "s":
            self.fwd = -1
            fwdR = -1
            fwdL = -1
        elif key == "a":
            l = 0
            r = 1024
        elif key == "d":
            r = 0
            l = 1024
        elif key == "q":
            fwdL = -1
            fwdR = 1
        elif key == "e":
            fwdL = 1
            fwdR = -1
        elif key == "x":
            fwdL = 0
            fwdR = 0
        elif key == "p":
            self.plot_occ_grid()
            fwdL = 0
            fwdR = 0
        elif key == "o":
            self.roombot.plot_recorded_history()
            fwdL = 0
            fwdR = 0
        elif key == "m":
            if self.roombot.disable_planning:
                self.roombot.disable_planning = 0
            else:
                self.roombot.disable_planning = 1

        ins = [l*fwdL, r*fwdR]
        self.roombot.ins_input = ins
        return ins

    def plot_occ_grid(self):
        """
        Plot the current occupancy grid in the pygame window. TODO inefficient
        """
        oc = self.roombot.occupancy_grid.copy()
        oc /= np.sum(oc) 
        oc /= np.max(oc)
        oc *= 255
        for x, row in enumerate(oc):
            for y, val in enumerate(row):
                if val != 0:
                    print(val)
                c_val = val*255
                if c_val >= 255:
                    color = (255, 255, 255)
                else: 
                    color = (c_val, c_val, c_val)
                # 1000-y because of weird pygame coordinate system
                pygame.draw.rect(self.display, color,(x,1000-y,1,1))
        pygame.display.update()
        #plt.matshow(self.roombot.occupancy_grid*255)
        #plt.show()
        

    def savelocdata(self, data1, data2, data3, phi, ins):
        """
        Save the sensor data to a csv file
        """
        self.logfile.write(";".join([str(x) for x in [time.time(), data1, data2, data3, phi, int(ins), "\n"]]))
        self.logfile.flush()
            
    def conv_ar_to_msg(self, ar):
        """
        Encode an instruction for the robot
        :param ar: Instruction to encode, [left, right]
        """
        if ar[0] > 1024:
            ar[0] = 1024
        if ar[0] < -1024:
            ar[0] = -1024
        if ar[1] > 1024:
            ar[1] = 1024
        if ar[1] < -1024:
            ar[1] = -1024

        pwrL = (ar[0]+1024).to_bytes(2, "little")
        pwrR = (ar[1]+1024).to_bytes(2, "little")
        msg = pwrL+pwrR
        return msg


class RoomBot:
    def __init__(self, log_data=0, disable_planning=0):
        """
        Roombot data handler
        :param log_data: Enable/Disable data logging TODO
        :param disable_planning: Disbale internal planning and use keyboard input instead
        """
        self.occupancy_grid = np.zeros((1000, 1000))
        self.position = [500,500]


        self.data_list = []
        self.ins_list = []
        self.log_data = log_data
        self.back_count = 0
        self.disable_planning = disable_planning
        self.ins_input = [0,0]
        self.ins_planned = [0,0]
        self.position_history = []

        # magnetometer offset
        self.mag_offset_X = -(103+160)/2
        self.mag_offset_Y = -(275+335)/2
        self.phi_offset = 0
        self.offset_count = 0
        self.phi_recent = 0

        # create a logfile if it doesnt exist
        if self.log_data:
            if not os.path.isfile("rb_datalog.csv"):
                with open("rb_datalog.csv", "w") as lf:
                    lf.write("t;s1;s2;s3;phi;ins;\n")
            self.logfile = open("rb_datalog.csv", "a")

    def __del__(self):
        if self.log_data:
            if self.logfile:
                self.logfile.close()
    
    def handle_mag_data(self, magX, magY):
        """
        Handle magnetometer data. Calculate angle and compensate device offset
        :param magX: Magnetometer value X direction
        :param magY: Magnetometer value Y direction
        """
        # offset due to manufactoring
        magX += self.mag_offset_X
        magY += self.mag_offset_Y

        # Scale because we are not at the equator
        magX *= 30/33
        magY *= 30/31
        
        # get angle from complex

        phi = np.arctan2(magX, magY)*180/np.pi

        phi -= self.phi_offset

        # -180:180 -> 0:360
        if phi < 0:
            phi += 360

        # zero magnetometer at start. First measurement is shit so skip it
        if self.offset_count == 1:
            self.offset_count += 1
            print(f"Offset magnetometer {phi} [DEG]")
            self.phi_offset = phi
        elif self.offset_count == 0:
            self.offset_count += 1
        self.phi_recent = phi

    def handle_dist_data(self, d1, d2, d3):
        """
        Clean distance data. (d1 front, d2 left, d3 right)
        :param d1: Front sensor distance value in cm
        :param d2: left sensor distance value in cm
        :param d3: right sensor distance value in cm
        """
        # center should be center of robot
        d1 += 12
        d2 += 7
        d3 += 7
        # sensor gives huge value of too close to wall
        if d1 > 2000:
            d1 = 1
        if d2 > 2000:
            d2 = 1
        if d3 > 2000:
            d3 = 1
        #print(d1, d2, d3, self.phi_recent, np.linalg.norm(self.position))
        self.data_new = [d1, d2, d3, self.phi_recent]

    def execute_timestep(self):
        """
        Execute a discrete robot timestep
        """
        # update position estimate according to shitty motion model
        self.check_phi_change()
        self.data_list.append(self.data_new)
        self.update_position()
        self.update_occupancy_grid()
        # plan if enabled
        if not self.disable_planning:
            self.plan_action()

        #if self.log_data:
        #    self.savelocdata(dist1, dist2, dist3, self.phi_recent, ins)
    
    def check_phi_change(self):
        last_ins = self.get_instruction()
        if last_ins in [[1024, 1024], [-1024, -1024]]:
            self.data_new[3] = 0.9*self.data_list[-1][3]+ 0.1*self.data_new[3]

    def update_occupancy_grid(self):
        x, y = self.position[0], self.position[1]
        if len(self.position_history) > 1:
            x_old, y_old = self.position_history[-2][0], self.position_history[-2][1]
        last_obs = self.data_list[-1]

        x1 = last_obs[0]*np.sin(last_obs[3]*np.pi/180) + x
        y1 = last_obs[0]*np.cos(last_obs[3]*np.pi/180) + y
        
        x2 = last_obs[1]*np.sin((last_obs[3]-90)*np.pi/180) + x
        y2 = last_obs[1]*np.cos((last_obs[3]-90)*np.pi/180) + y
        x3 = last_obs[2]*np.sin((last_obs[3]+90)*np.pi/180) + x
        y3 = last_obs[2]*np.cos((last_obs[3]+90)*np.pi/180) + y

        #self.set_occgrid_coord(x,y, 1)
        #self.set_occgrid_coord(x_old,y_old, 1)
        self.set_occgrid_coord(x1,y1, last_obs[0])
        self.set_occgrid_coord(x2,y2, last_obs[1])
        self.set_occgrid_coord(x3,y3, last_obs[2])

    def set_occgrid_coord(self, x ,y, val=1):
        """
        Put an observation into the occupancy grid in form of a gaussian blob
        :param x: x coordinate of observation
        :param y: y coordinate of observation
        :param val: base value for how certain this observation is
        """
        # sensors suck for big distances
        if val < 100:
            return
        sigma = (val/50)
        mu = 0
        def blobfunc(x):
            return (1/(sigma*np.sqrt(2*np.pi)))*np.exp(-0.5*((x-mu)**2/sigma**2))

        blobsize = 10
        if blobsize <= x < 1000-blobsize and blobsize <= y < 1000-blobsize:
            for dx in range(-blobsize, blobsize):
                for dy in range(-blobsize, blobsize):
                    if np.sqrt(dy**2+dx**2) < blobsize:
                        up_val = blobfunc(np.sqrt(dx**2+dy**2))
                        self.occupancy_grid[int(round(x+dx))][int(round(y+dy))] += up_val

                
    
    def update_position(self):
        """
        Update the robots estimated position based on a shitty motion model
        """
        last_ins = self.get_instruction()
        last_obs = self.data_list[-1]

        # robot velocity in 1/100 cm/s
        v_robot = 5.5
        dx = 0
        dy = 0
        if last_ins == [1024, 1024]:
            dx = v_robot*np.sin(last_obs[3]*np.pi/180)
            dy = v_robot*np.cos(last_obs[3]*np.pi/180)
        elif last_ins == [-1024, -1024]:
            dx = -1 * v_robot*np.sin(last_obs[3]*np.pi/180)
            dy = -1 * v_robot*np.cos(last_obs[3]*np.pi/180)
        elif last_ins in [[1024, -1024], [-1024, 1024]]:
            dx = 0
            dy = 0
        else:
            dx = last_ins[0]/1024* v_robot*np.sin(last_obs[3]*np.pi/180)
            dx = last_ins[1]/1024* v_robot*np.sin(last_obs[3]*np.pi/180)
        self.position[0] += dx
        self.position[1] += dy
        self.position_history.append(np.copy(self.position))

    def plan_action(self):
        """
        React to new sensordata.
        Basically modify the instruction list
        """
        d1, d2, d3, phi = self.data_list[-1]
        
        r = 1024
        l = 1024
        # dont crash into walls
        if d1 < 50 and (d2 > 20 or d3 > 20):
            if d2 > d3:
                l *= -1
            elif d3 >d2:
                r *= -1
            # randomly decide left/right if no clear choice
            elif d3==d2:
                l, r = random.choice([[-1024, 1024], [1024, -1024]])
        # if close to left left go right
        elif d2 < 20 and d3 > 20:
            r *= -1
        # if close to right go left
        elif d3 < 20 and d2 > 20:
            l *= -1
        # if stuck go back
        elif d3 < 20 and d2 < 20 and d1 < 40:
            l *= -1
            r *= -1
        else:
            # correct for driving a straight line using the gradient
            if len(self.data_list) >= 5:
                d1_dt= np.mean(np.diff([x[0] for x in self.data_list[-5:]]))
                d2_dt= np.mean(np.diff([x[1] for x in self.data_list[-5:]]))
                d3_dt= np.mean(np.diff([x[2] for x in self.data_list[-5:]]))
                # if not moving the gradient is undefined
                if np.abs(d1_dt) > 0.01:
                    # left and right are redundant, sensors are better at close distance, use the closer one
                    if d3 > d2:
                        # d2/d1 = d2/dt * dt/d1
                        grad = d2_dt/d1_dt
                    else:
                        grad = -d3_dt/d1_dt
                    # correct course based on gradient
                    if grad > 0:
                        r -= grad*500
                    else:
                        l -= grad*500
        self.ins_planned = [int(l),int(r)]
        self.ins_list.append([l,r])

    def get_instruction(self):
        """
        Get the next instruction for the robot.
        In manual mode this returns the last keyboard instruciton
        In planning mode this returns the next action according to plan
        """
        if self.disable_planning:
            ins = self.ins_input
        else:
            ins = self.ins_planned
           #if len(self.ins_list):
           #    ins = self.ins_list[-1]
           #else:
           #    ins = [0,0]
        return ins



    def plot_recorded_history(self):
        """
        Plot robots position and relative sensor data
        """
        x_coord = np.array([x[0] for x in self.position_history])
        y_coord = np.array([x[1] for x in self.position_history])
        x1 = np.array([data[0]*np.sin(data[3]*np.pi/180) for data in self.data_list]) + x_coord
        y1 = np.array([data[0]*np.cos(data[3]*np.pi/180) for data in self.data_list]) + y_coord
        x2 = np.array([data[1]*np.sin((data[3]-90)*np.pi/180) for data in self.data_list]) + x_coord
        y2 = np.array([data[1]*np.cos((data[3]-90)*np.pi/180) for data in self.data_list]) + y_coord
        x3 = np.array([data[2]*np.sin((data[3]+90)*np.pi/180) for data in self.data_list]) + x_coord
        y3 = np.array([data[2]*np.cos((data[3]+90)*np.pi/180) for data in self.data_list]) + y_coord
        
        #plt.matshow(self.occupancy_grid*255)
        #plt.show()
        plt.scatter(x_coord, y_coord)
        #plt.scatter(x1, y1)
        #plt.scatter(x2, y2)
        #plt.scatter(x3, y3)
        plt.xlim(0, 1000)
        plt.ylim(0, 1000)
        plt.show()


if __name__ == "__main__":
    # manual mode command line argument
    manual = 0
    if len(sys.argv) > 1:
        if sys.argv[1] == "manual":
            manual = 1
    # mqtt broker port
    port = 1883
    # mqtt broker ip
    ip = "192.168.178.27"
    # setup pygame for manual mode
    hs = SMFCBase(ip, port, manual=manual)
    hs.serve()
