import time
import os, sys
import numpy as np
import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import getch
import pygame
import pygame.locals


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
        robot_clock = 0.2
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
            self.roombot.plot_recorded_history()

        ins = [l*fwdL, r*fwdR]
        self.roombot.ins_input = ins
        return ins


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
        print(d1, d2, d3, self.phi_recent, np.linalg.norm(self.position))
        self.data_new = [d1, d2, d3, self.phi_recent]

    def execute_timestep(self):
        """
        Execute a discrete robot timestep
        """
        # plan if enabled
        if not self.disable_planning:
            self.plan_action()
        # update position estimate according to shitty motion model
        self.data_list.append(self.data_new)
        self.update_position()

        #if self.log_data:
        #    self.savelocdata(dist1, dist2, dist3, self.phi_recent, ins)

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
        if last_ins == [-1024, -1024]:
            dx = -1 * v_robot*np.sin(last_obs[3]*np.pi/180)
            dy = -1 * v_robot*np.cos(last_obs[3]*np.pi/180)
        if last_ins in [[1024, 0], [0, 1024]:
            dx = 0.5 * v_robot*np.sin(last_obs[3]*np.pi/180)
            dy = 0.5 * v_robot*np.cos(last_obs[3]*np.pi/180)
        if last_ins in [[-1024, 0], [0, -1024]:
            dx = -0.5 * v_robot*np.sin(last_obs[3]*np.pi/180)
            dy = -0.5 * v_robot*np.cos(last_obs[3]*np.pi/180)
        self.position[0] += dx
        self.position[1] += dy
        self.position_history.append(np.copy(self.position))

    def plan_action(self):
        """
        React to new sensordata.
        Basically modify the instruction list
        """
        # for now only plan on most recent data
        d1, d2, d3, phi = self.data_list[-1]
        r = 1024
        l = 1024
        bc_1 = [[-1*l, -1*r]]
        bc_2 = [[-1*l, 1*r]]
        bc_lookup = bc_2 * 10 + bc_1 * 10
        bc_max = len(bc_lookup)
        if d1 < 30 or self.back_count > 0:
            if bc_max >= self.back_count > 0:
                self.back_count -= 1
                ins = bc_lookup[self.back_count]
            else:
                self.back_count = bc_max
                ins = [-1*l, -1*r]
        else:
            ins = [1*l, 1*r]
        self.ins_list.append(ins)

    def get_instruction(self):
        """
        Get the next instruction for the robot.
        In manual mode this returns the last keyboard instruciton
        In planning mode this returns the next action according to plan
        """
        if self.disable_planning:
            ins = self.ins_input
        else:
            if len(self.ins_list):
                ins = self.ins_list[-1]
            else:
                ins = [0,0]
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
        
        plt.scatter(x_coord, y_coord)
        plt.scatter(x1, y1)
        plt.scatter(x2, y2)
        plt.scatter(x3, y3)
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
    if manual:
        pygame.init()
        BLACK = (0,0,0)
        WIDTH = 1280
        HEIGHT = 1024
        windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)
        windowSurface.fill(BLACK)

    hs = SMFCBase(ip, port, manual=manual)
    hs.serve()
