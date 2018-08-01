import socket

TCP_IP = '0.0.0.0'
TCP_PORT = 3000

class TCPDataSender:
    def __init__(self, port=3000):
        self.port = port

    def start(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((TCP_IP, self.port))
        self.sock.listen(1)
        self.conn, self.addr = self.sock.accept()

    def stop(self):
        self.sock.close()
        if not self.conn:
            return

        self.conn.close()

    def send_data(self, data):
        if not data or not self.conn:
            return
        self.conn.send(str.encode(data))

if __name__ == '__main__':
    tcp_data_sender = TCPDataSender()
    tcp_data_sender.start()
    tcp_data_sender.send_data('test_data')
    tcp_data_sender.stop()