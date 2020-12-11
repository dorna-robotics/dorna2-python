import asyncio
import threading
import queue
import json


class ws(object):
    """docstring for ws"""
    def __init__(self, read_len=1000):
        super(ws, self).__init__()
        self.read_len = read_len

        self.read_q = queue.Queue()
        self.write_q = queue.Queue()
        self.msg = queue.Queue(100)
        self.sys = {}

        self.connected = False
    
    def connect_loop(self, ip, port):
        asyncio.run(self.async_connect_loop(ip, port))

    async def _handshake(self):
        message = (
            b"GET /chat HTTP/1.1\r\n"
            b"Host: 192.168.1.8:443\r\n"
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
            b"Sec-WebSocket-Extensions: permessage-deflate; \
            client_max_window_bits \r\n"
            b"\r\n"
            )
        self.writer.write(message)
        await self.writer.drain()

    async def async_connect_loop(self, ip, port):
        try:
            # establish connection
            self.reader, self.writer = await asyncio.open_connection(ip, port)
            await self._handshake()
            self.connected = True

            # read thread
            self.read_thread = threading.Thread(target=self.read_loop)
            self.read_thread.start()

            # connection loop
            while self.connected:

                # write message
                if not self.write_q.empty():
                    self.writer.write(self.write_q.get())
                    await self.writer.drain()

                # read message
                data = await self.reader.read(self.read_len)
                if len(data):
                    self.read_q.put(data)

        except Exception as ex:
            print(ex)
        finally:
            await self.close()

    async def close(self):
        self.writer.close()
        await self.writer.wait_closed()
        self.connected = False

    """
    decode the read data in read_q
    """
    def read_loop(self):
        # initialize
        state = 0
        buff = b""
        waiting_len = 1

        while self.connected:
            # add to the buffer
            while not self.read_q.empty():
                buff += self.read_q.get()

            if waiting_len > len(buff):  # make sure buffer has enough length
                continue

            elif state == 0:
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
                    buff = buff[waiting_len:]
                    waiting_len = buff[0]
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
                    if not self.msg.full():
                        self.msg.put(msg)
                    self.sys = {**dict(self.sys), **msg}  # update sys
                except:
                    pass

                state = 0
                buff = buff[waiting_len_tmp:]
                waiting_len = 1

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

        elif "mode" == "handshake":
            self.write_q.put((
                b"GET /chat HTTP/1.1\r\n"
                b"Host: 192.168.1.8:443\r\n"
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

    def connect(self, ip, port):

        self.connect_thread = threading.Thread(
            target=self.connect_loop, args=(ip, port)
        )
        self.connect_thread.start()
        return self.connected


def main():
    web_socket = ws()
    web_socket.connect("192.168.1.7", 443)

if __name__ == '__main__':
    main()
