import socketserver
import time


class TCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data1 = self.request.recv(1024).strip().decode()
        self.data2 = self.request.recv(1024).strip().decode()
        #self.data = self.request.recv(1024).strip().decode()
        #self.data = self.request.recv(1024).strip().decode()
        #print("{} wrote:".format(self.client_address[0]))
        #self.request.send(b'alive')
        print(self.data1, self.data2)
        if self.data1 == "LOCDATA":
            self.handle_locdata()

    def handle_locdata(self):
        while 1:
            #time_last_rcv = time.time()
            self.data = self.request.recv(1024).strip().decode()
            print(self.data)



if __name__ == "__main__":
    host_ip, port = "0.0.0.0", 9999
    with socketserver.TCPServer((host_ip, port), TCPHandler) as server:
        server.serve_forever()
