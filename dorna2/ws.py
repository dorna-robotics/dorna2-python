import threading
import queue
import json
import time
import asyncio


class WS(object):
    """docstring for comm"""
    def __init__(self, channel="websocket"):
        super(WS, self).__init__()
        self.channel = channel
        self.msg = queue.Queue(100)
        self._sys = {}
        self.deregister_callback()
        self._connected = False
        
        # wait
        self._ptrn = {"wait": None, "sys": None} # wait for a given pattern
        self._track = {"id": None, "msgs": [], "cmd": {}} # track a given id until it is done 
        self._recv = {} # last message recived

        # last message sent
        self._send = {}
    """
    server 
    """
    async def server_init(self, ip, port, timeout):

        handshake = False
        self._connected = False

        # define the loop
        self.loop = asyncio.get_running_loop()            

        # reader and writer
        try:
            self.reader, self.writer = await asyncio.wait_for(asyncio.open_connection(ip, port), timeout=1)
        except:
            return await self.close_coro()

        # handshake
        if self.channel == "websocket":                
            try:
                handshake = await asyncio.wait_for(self._handshake_ws(), timeout=timeout)
            except asyncio.TimeoutError:
                return await self.close_coro()
        else:
            handshake = True
        
        # read loop
        if handshake:
            await self.read_loop()
        
        await self.close_coro()


    def server(self, ip, port, timeout=5):
        # start a new thread
        self.server_thread = threading.Thread(target=asyncio.run, args=(self.server_init(ip, port, timeout),))
        self.server_thread.start()

        # check for the connection
        self._thr = True
        s = time.time()
        while time.time()-s < timeout:
            if self._connected or not self._thr:
                break
            time.sleep(0.001)
        
        return self._connected        

    def write(self, msg = "", mode="cmd"):
        #asyncio.create_task(self.write_coro(msg, mode))
        future = asyncio.run_coroutine_threadsafe(self.write_coro(msg, mode), self.loop)

    # write coroutine
    async def write_coro(self, msg="", mode="cmd"):
        try:
            # msg to byte
            msg_byte = self.write_process(msg, mode)            
            self.writer.write(msg_byte)
            await self.writer.drain()
            return True
        except:
            pass
        return False


    # register a callback
    def register_callback(self, fn):
        ''' fn must accept one input, e.g. fn(msg, sys) '''
        self.callback = fn


    def deregister_callback(self):
        self.register_callback(None)


    async def _handshake_ws(self):
        try:
            # send the handshake and wait for its reply
            await self.write_coro("", mode="handshake")
            data = str(await self.reader.readuntil(separator=b'\r\n\r\n'))
            if "Sec-WebSocket-Accept" in data:
                return True

        except:
            pass
        return False


    # read loop
    async def read_loop(self):
        sys = {}
        self._connected = True
        while self._connected:
            msg = None
            try:
                if self.channel == "websocket":
                    # raw data
                    try:
                        data_byte = await self.reader.readuntil(separator=b'}')
                        data_str = str(data_byte)
                    except Exception as ex:
                        # close connection
                        break

                    # find the index
                    index_start = data_str.find("{")

                    # get the message
                    msg = json.loads(data_str[index_start:-1])
                else:
                    try:
                        # read header
                        data_length_byte = await self.reader.readexactly(2)

                        # read data
                        data_byte = await self.reader.readexactly(int.from_bytes(data_length_byte, byteorder='big'))
                    except:
                        break
                       
                    # get the message
                    msg = json.loads(data_byte.decode("utf-8") )                    
                
                # message queue
                if not self.msg.full():
                    self.msg.put(msg)
                else:
                    self.msg.get()
                    self.msg.put(msg)

                # update _msg
                self._recv = dict(msg)

                # update sys
                sys.update(msg)

                # callback
                if self.callback:
                    asyncio.create_task(self.callback(msg.copy(), sys.copy()))
                

                # track a given id
                if self._track["id"]:
                    try:
                        # message contains an id
                        if "id" in msg and self._track["id"] == msg["id"]:
                            # update the resp_id
                            self._track["msgs"].append(dict(msg))

                            # end the track
                            if "stat" in msg and any([msg["stat"] < 0, msg["stat"] >= 2]):
                                self._track["id"] = None                              
                    except:
                        pass

                # pattern wait
                try:
                    if type(self._ptrn["wait"])== dict:
                        # update sys
                        self._ptrn["sys"] = dict(sys)
                        # search for it
                        if all([k in sys for k in self._ptrn["wait"]]) and all([self._ptrn["wait"][k] == sys[k] for k in self._ptrn["wait"]]):
                            self._ptrn["wait"] = None
                except:
                    pass 

                # update sys           
                self._sys = dict(sys)


            except Exception as ex:
                print("socket read error: ", ex)

        # close connection
        await self.close_coro()

    # encode the write data
    def write_process(self, msg, mode):
        # handle socket
        if self.channel == "socket":
            return (len(msg)).to_bytes(2, byteorder='big') + msg.encode("utf-8")

        # handle websocket
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

    async def close_coro(self):
        if self._connected:
            try:
                self.writer.close()
                await self.writer.wait_closed()
            except:
                pass
        self._connected = False
        self._thr = False

    def close(self):
        if self._connected:
            future = asyncio.run_coroutine_threadsafe(self.close_coro(), self.loop)
        #asyncio.run(self.close_coro())
