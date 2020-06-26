import socketserver


class TCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        # self.request is the TCP socket connected to the client
        self.data = self.request.recv(1024).strip()
        print("{} wrote:".format(self.client_address[0]))
        print(self.data)
        # just send back the same data, but upper-cased
        #self.request.sendall(self.data.upper())


class ShittyRobotServer:
    def __init__(self, port=5005, ip="0.0.0.0"):
        TCP_IP = '0.0.0.0'
        TCP_PORT = 5005
        BUFFER_SIZE = 1024  # Normally 1024, but we want fast response

        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)
        conn, addr = s.accept()
        print('Connection address:', addr)

if __name__ == "__main__":
    host_ip, port = "0.0.0.0", 9999
    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((host_ip, port), TCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
