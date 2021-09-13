import socket
import threading
import queue
import json
import time
import asyncio


class ws(object):
    """docstring for comm"""
    def __init__(self, read_len=10):
        super(ws, self).__init__()
        self.read_len = read_len
        self.write_q = queue.Queue()
        self.msg = queue.Queue(100)
        self.id_q = queue.Queue(100)
        self.sys = {}
        self.connected = False
        self.ptrn_wait = None

        # read queue
        self.read_q = asyncio.Queue()        


    """
    server 
    """
    async def server_init(self, ip, port):
        try:
            self.connected = False
            # connect
            # define the loop
            self.loop = asyncio.get_running_loop()            
            self.reader, self.writer = await asyncio.open_connection(ip, port)
            
            # handshake
            self.write(mode="handshake")
            #self.writer.write(self.write_process("", "handshake"))
            #await self.writer.drain()
            # set parameter
            await asyncio.sleep(1)
            self.connected = True

            # gather reader and writer
            await asyncio.gather(self.read_loop())
        except Exception as ex:
            self.connected = False
            print("connection error: ", ex)

    def server(self, ip, port, time_out=5):
        # close the previous open servers???

        # start a new thread
        self.server_thread = threading.Thread(target=asyncio.run, args=(self.server_init(ip, port),))
        self.server_thread.start()

        # check for the connection
        s = time.time()
        while time.time()-s < time_out:
            if self.connected:
                break
        return self.connected        

    def write(self, msg = "", mode="cmd"):
        # msg to byte
        msg_byte = self.write_process(msg, mode)

        #submit the coroutine to the given loop
        future = asyncio.run_coroutine_threadsafe(self.write_coro(msg_byte), self.loop)
    
    # write coroutine
    async def write_coro(self, msg_byte):
        self.writer.write(msg_byte)
        await self.writer.drain()
        await asyncio.sleep(0.001)
        return True

    # read loop
    async def read_loop(self):
        state = 0
        buff = b""
        waiting_len = 1
        sys = {}
        while self.connected:
            try:
                data = await self.reader.read(self.read_len)
            except Exception as ex:
                print("socket read error: ", ex)
            
            if len(data):                
                # add to the buffer
                buff += data
                
                # process buffer
                buff, waiting_len, state, msg = self.read_process(buff, waiting_len, state)
                if msg:
                    # message queue
                    if not self.msg.full():
                        self.msg.put(msg)
                    else:
                        self.msg.get()
                        self.msg.put(msg)
                    

                    # update sys
                    sys = {**dict(sys), **msg}
                    
                    # check wait ??? handle alarm and halt
                    if type(self.ptrn_wait) is dict:
                        try:
                            if all([sys[x] == self.ptrn_wait[x] for x in self.ptrn_wait]):
                                print("received message: ", msg)
                                self.ptrn_wait = None
                        except Exception as ex:
                            #print("Pattern waiting error: ",ex)
                            pass
                    
                    # update sys           
                    self.sys = dict(sys)
            
            await asyncio.sleep(0)


    # encode the write data
    def write_process(self, msg, mode):
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

            return bytes(header + mask) + msg.encode("utf-8")

        elif mode == "handshake":
            return (
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
            )

    """
    decode the read data
    """
    def read_process(self, buff, waiting_len, state):
        msg = {}
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
                except Exception as ex:
                    print("buffer_error: ", ex)
                    pass

                state = 0
                buff = buff[waiting_len:]
                waiting_len = 1
                self.b = len(buff)

        return buff, waiting_len, state, msg

    async def close_coro(self):
        self.connected = False
        try:
            self.writer.close()
            await self.writer.wait_closed()
        except:
            pass

    def close(self):
        #submit the coroutine to the given loop
        future = asyncio.run_coroutine_threadsafe(self.close_coro(), self.loop)


    """
    wait for a given pattern
    ptrn = None
    ptrn = True
    ptrn = {}
    """
    def wait(self, time_out=0, **ptrn):
        self.ptrn_wait = dict(ptrn)
        if time_out > 0:
            start = time.time()
            while time.time() <= start + time_out:
                if self.ptrn_wait != ptrn:
                    return ptrn
                time.sleep(0.001)

        else:
            while True:
                if self.ptrn_wait != ptrn:
                    return ptrn 
                time.sleep(0.001)

        self.ptrn_wait = None
        return dict()


def main():
    ip = "192.168.1.8"
    port = 443

    web_socket = ws()
    web_socket.server(ip, port)
    print("ready to write")

    for i in range(10):
        cmd = {"cmd": "uid", "id": i+2} 
        web_socket.write(json.dumps(cmd))
        time.sleep(0.001)

if __name__ == '__main__':
    main()
