import socketserver
import time
back_count = 0

class TCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data1 = self.request.recv(4)
        self.data1_fl = float(self.data1.strip().decode())/10
        #self.data2 = self.request.recv(1024).strip().decode()
        ins = self.handle_locdata(self.data1_fl)
        self.request.send(ins)
        print(self.data1_fl)

    def handle_locdata(self, dat1):
        global back_count
        # 5 stop and go cycles -> ~180 deg turn
        bc_1 = [b"2", b"2", b"0", b"0"] * 2
        bc_2 = [b"2", b"0"]*10
        bc_lookup = [b"2"]*10 + bc_2 + bc_1
        bc_max = len(bc_lookup)
        if dat1 < 22 or back_count > 0:
            if bc_max >= back_count > 0:
                back_count -= 1
                ins = bc_lookup[back_count]
            else:
                back_count = bc_max
                ins = b"0"
        else:
            ins = b"2"
        return ins



if __name__ == "__main__":
    host_ip, port = "0.0.0.0", 9999
    with socketserver.TCPServer((host_ip, port), TCPHandler) as server:
        server.serve_forever()
