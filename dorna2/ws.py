import socket
import threading
import queue
import json
import time


class ws(object):
    def __init__(self, read_len=10000):
        super(ws, self).__init__()

        self.read_len = read_len
        self.write_q = queue.Queue()
        self.msg = queue.Queue(100)
        self.id_q = queue.Queue(100)
        self.sys = {}
        self.connected = False

    def socket_loop(self):
        # initialize read buffer
        state = 0
        buff = b""
        waiting_len = 1

        while self.connected:
            time.sleep(0)
            # write 2 commands
            for _ in range(2):
                if not self.write_q.empty():
                    msg = self.write_q.get()
                    self.sock.send(msg)
            # read
            try:
                # read data
                data = self.sock.recv(self.read_len)

                if len(data):
                    # add to the buffer
                    buff += data
                    # process buffer
                    buff, waiting_len, state = self.process_buffer(buff, waiting_len, state)
            except socket.error as error:
                pass

        self.close()

    def close(self):
        self.connected = False
        self.sock.close()

    """
    decode the read data
    """
    def process_buffer(self, buff, waiting_len, state):
        while len(buff) >= waiting_len:  # make sure buffer has enough length
            if state == 0:
                if buff[0] == 129:  # message start
                    state = 21
                    buff = buff[waiting_len:]
                    waiting_len = 1

                else:  # basically remove the byte
                    state = 0
                    buff = buff[waiting_len:]
                    waiting_len = 1

            elif state == 21:  # length type
                if buff[0] <= 125:
                    state = 25
                    waiting_len = buff[0]
                    buff = buff[1:]

                elif buff[0] == 126:
                    state = 22
                    buff = buff[waiting_len:]
                    waiting_len = 2
                else:
                    state = 23
                    buff = buff[waiting_len:]
                    waiting_len = 8

            elif state == 22 or state == 23:
                state = 25
                waiting_len_tmp = waiting_len
                waiting_len = sum([
                    int(buff[i] << (8*(waiting_len_tmp-i-1)))
                    for i in range(waiting_len_tmp)
                ])
                buff = buff[waiting_len_tmp:]

            elif state == 25:  # add to the message queue
                try:
                    msg = json.loads(str(buff[:waiting_len], "utf-8"))

                    # message queue
                    if not self.msg.full():
                        self.msg.put(msg)
                    else:
                        self.msg.get()
                        self.msg.put(msg)

                    # track queue
                    if "id" in msg:
                        self.id_q.put(msg)

                    # update sys
                    self.sys = {**dict(self.sys), **msg}  # update sys
                except Exception as ex:
                    pass

                state = 0
                buff = buff[waiting_len:]
                waiting_len = 1
                self.b = len(buff)

        return buff, waiting_len, state

    """
    encode the write data and add it to write_q
    """
    def send(self, msg="", mode="cmd"):
        if mode == "cmd" and msg:
            # cmd command
            header = [129]
            mask = [0, 0, 0, 0]
            msg_len = len(msg)

            if msg_len <= 125:
                header.append(msg_len + 128)

            elif msg_len < 0xffffffff:
                # header length
                header.append(126 + 128)
                header.append((msg_len << 0) >> 8)
                header.append((msg_len << 8) >> 8)

            else:
                # header length
                header.append(127 + 128)
                header.append((msg_len << 0) >> 56)
                header.append((msg_len << 8) >> 56)
                header.append((msg_len << 16) >> 56)
                header.append((msg_len << 24) >> 56)
                header.append((msg_len << 32) >> 56)
                header.append((msg_len << 40) >> 56)
                header.append((msg_len << 48) >> 56)
                header.append((msg_len << 56) >> 56)

            self.write_q.put(bytes(header + mask) + msg.encode("utf-8"))

        elif mode == "handshake":
            self.write_q.put((
                b"GET /chat HTTP/1.1\r\n"
                b"Host: localhost:443\r\n"
                b"Connection: Upgrade\r\n"
                b"Pragma: no-cache\r\n"
                b"Cache-Control: no-cache\r\n"
                b"User-Agent: api\r\n"
                b"Upgrade: websocket\r\n"
                b"Origin: localhost\r\n"
                b"Sec-WebSocket-Version: 13\r\n"
                b"Accept-Encoding: gzip, deflate\r\n"
                b"Accept-Language: en-US,en;q=0.9,fa;q=0.8\r\n"
                b"Sec-WebSocket-Key: x3JJHMbDL1EzLkh9GBhXDw==\r\n"
                b"Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n"
                b"\r\n"
            ))

    def _connect(self, host, port, time_out=1):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

        self.connected = True
        # handshake
        self.send(mode="handshake")

        # socket thread
        self.socket_thread = threading.Thread(
            target=self.socket_loop
        )
        self.socket_thread.start()

        time.sleep(time_out)
        self.sock.setblocking(0)


def main():
    web_socket = ws()
    web_socket._connect("dorna", 443)

if __name__ == '__main__':
    main()
