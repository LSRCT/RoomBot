import socketserver


class TCPHandler(socketserver.BaseRequestHandler):

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip().decode()
        print("{} wrote:".format(self.client_address[0]))
        if self.data == "LOCDATA":
            self.handle_locdata()

    def handle_locdata(self):
        while 1:
            self.data = self.request.recv(1024).strip().decode()
            print(self.data)
            if self.data[0] != "L":
                break


if __name__ == "__main__":
    host_ip, port = "0.0.0.0", 9999
    with socketserver.TCPServer((host_ip, port), TCPHandler) as server:
        server.serve_forever()
