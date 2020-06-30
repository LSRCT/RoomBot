import socketserver
import time
back_count = 0

class TCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data1 = float(self.request.recv(10).strip().decode())
        #self.data2 = self.request.recv(1024).strip().decode()
        #self.data = self.request.recv(1024).strip().decode()
        #self.data = self.request.recv(1024).strip().decode()
        #print("{} wrote:".format(self.client_address[0]))
        ins = self.handle_locdata(self.data1)
        self.request.send(ins)
        print(self.data1)
        #if self.data1 == "LOCDATA":
        #    self.handle_locdata()

    def handle_locdata(self, dat1):
        global back_count
        bc_max = 6

        if dat1 < 22 or back_count > 0:
            if back_count == bc_max:
                back_count -= 1
                ins = b"0"
            elif bc_max > back_count > 0:
                back_count -= 1
                ins = b"1"
            else:
                back_count = bc_max
                ins = b"2"
        else:
            ins = b"2"
        return ins



if __name__ == "__main__":
    host_ip, port = "0.0.0.0", 9999
    with socketserver.TCPServer((host_ip, port), TCPHandler) as server:
        server.serve_forever()
