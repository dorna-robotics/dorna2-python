import threading
import queue
import json
import time
import asyncio
import copy
import random
import base64
import os

class WS(object):
    """docstring for comm"""
    def __init__(self, channel="websocket"):
        super(WS, self).__init__()
        self.channel = channel
        self.msg = queue.Queue(100)
        self._sys = {}
        self.deregister_callback()
        self._event_list = [] # event list [{"target":fn0, "kwargs":kwargs0},...,{"target":fnn, "kwargs":kwargs0}]
        self._connected = False
        
        # wait
        self._ptrn = {"wait": None, "sys": None} # wait for a given pattern
        self._track = {"id": None, "msgs": [], "cmd": {}} # track a given id until it is done 
        self._recv = {} # last message recived

        # last message sent
        self._send = {}

        # loop and socket
        self.loop = None
        self.reader = None
        self.writer = None

        # emergency
        self._emergency = {"enable": False, "key": "in0", "value":1}
        self._emergency_flag = False

    """
    server 
    """
    async def server_init(self, ip, port, timeout):

        handshake = False

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
        
        return await self.close_coro()


    def server(self, ip, port, timeout=5):
        # start a new thread
        self.server_thread = threading.Thread(target=asyncio.run, args=(self.server_init(ip, port, timeout),))
        self.server_thread.start()

        # check for the connection
        s = time.time()
        while time.time()-s < timeout:
            time.sleep(0.001)
            if self._connected or not self.server_thread.is_alive():
                break
        
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
        except Exception as ex:
            pass
        return False


    # register a callback
    def register_callback(self, fn):
        ''' fn must accept one input, e.g. fn(msg, sys) '''
        self.callback = fn


    def deregister_callback(self):
        self.register_callback(None)


    def get_all_event(self):
        return list(self._event_list)

    # register event
    def add_event(self, target=None, kwargs={}, index=None):        
        # find the index
        if index is None:
            index = len(self._event_list)

        if target is not None:
            ''' fn must accept one input, e.g. fn(msg, union, **kwargs) '''
            self._event_list.insert(index, {"target":target, "kwargs":kwargs})
        
        return self.get_all_event()

    def clear_all_event(self):
        self._event_list.clear()
        return self.get_all_event()

    # first look for the index and then fn
    def clear_event(self, target=None):
        if target is not None:
            try:
                self._event_list.pop([event["target"] for event in self._event_list].index(target))
            except:
                pass
        return self.get_all_event()



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
                    if index_start < 0:
                        continue

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
                    msg = json.loads(data_byte.decode("utf-8"))                    
                
                # message queue
                if not self.msg.full():
                    self.msg.put(msg)
                else:
                    self.msg.get()
                    self.msg.put(msg)

                # update _msg
                self._recv = copy.deepcopy(msg)

                # update sys
                sys.update(msg)

                # callback
                if self.callback:
                    asyncio.create_task(self.callback(copy.deepcopy(msg), copy.deepcopy(sys)))

                # emergency
                if self._emergency["enable"] and self._emergency["key"] in ["in"+str(i) for i in range(16)] and self._emergency["value"] in [i for i in range(2)]:
                    # activate emergency
                    if not self._emergency_flag and self._emergency["key"] in msg and msg[self._emergency["key"]] == self._emergency["value"]:
                        msg = {"cmd":"alarm", "alarm":1, "id":100+random.randint(1,10)}
                        asyncio.create_task(self.write_coro(json.dumps(msg)))
                        self._emergency_flag = True
                    
                    elif self._emergency_flag and self._emergency["key"] in msg and msg[self._emergency["key"]] != self._emergency["value"]:
                        msg = {"cmd":"alarm", "alarm":0, "id":100+random.randint(1,10)}
                        asyncio.create_task(self.write_coro(json.dumps(msg)))
                        self._emergency_flag = False                    

                # track a given id
                if self._track["id"]:
                    try:
                        # message contains an id
                        if "id" in msg and self._track["id"] == msg["id"]:
                            # update the resp_id
                            self._track["msgs"].append(copy.deepcopy(msg))

                            # end the track
                            if "stat" in msg and any([msg["stat"] < 0, msg["stat"] >= 2]):
                                self._track["id"] = None                              
                    except:
                        pass

                # events
                for event in self._event_list:
                    try:
                        #asyncio.create_task(event["target"](msg=copy.deepcopy(msg), union=copy.deepcopy(sys), **event["kwargs"]))
                        asyncio.create_task(asyncio.to_thread(event["target"], copy.deepcopy(msg), copy.deepcopy(sys), **event["kwargs"]))

                    except Exception as ex:
                        # clear the event
                        self.clear_event(event["target"])

                # pattern wait
                try:
                    if type(self._ptrn["wait"])== dict:
                        # update sys
                        self._ptrn["sys"] = copy.deepcopy(sys)
                        # search for it
                        if all([k in sys for k in self._ptrn["wait"]]) and all([self._ptrn["wait"][k] == sys[k] for k in self._ptrn["wait"]]):
                            self._ptrn["wait"] = None
                except:
                    pass 

                # update sys           
                self._sys = copy.deepcopy(sys)


            except Exception as ex:
                print("socket read error: ", ex)

        # close connection
        await self.close_coro()

    # encode the write data
    def write_process(self, msg, mode):
        # Handle socket (non-WebSocket)
        if self.channel == "socket":
            return len(msg).to_bytes(2, byteorder='big') + msg.encode("utf-8")

        # Handle WebSocket
        if mode == "cmd" and msg:
            # WebSocket frame construction
            header = [0x81]  # FIN=1, Opcode=1 (text frame)
            msg_len = len(msg)
            mask = [random.getrandbits(8) for _ in range(4)]  # Random mask

            # Handle payload length
            if msg_len <= 125:
                header.append(msg_len | 0x80)  # Set mask bit
            elif msg_len <= 0xFFFF:
                header.extend([126 | 0x80, (msg_len >> 8) & 0xFF, msg_len & 0xFF])
            else:
                header.append(127 | 0x80)
                for shift in [56, 48, 40, 32, 24, 16, 8, 0]:
                    header.append((msg_len >> shift) & 0xFF)

            # Add mask to header
            header.extend(mask)
            
            # Mask the payload
            payload_bytes = msg.encode('utf-8')
            masked_payload = bytes([b ^ mask[i % 4] for i, b in enumerate(payload_bytes)])
            
            return bytes(header) + masked_payload

        elif mode == "handshake":
            # Generate random WebSocket key
            key = base64.b64encode(os.urandom(16)).decode()
            
            # Proper HTTP handshake request
            return (
                f"GET /chat HTTP/1.1\r\n"
                f"Host: localhost:443\r\n"
                f"Connection: Upgrade\r\n"
                f"Upgrade: websocket\r\n"
                f"Sec-WebSocket-Version: 13\r\n"
                f"Sec-WebSocket-Key: {key}\r\n"
                f"Origin: http://localhost\r\n"
                f"\r\n"
            ).encode('utf-8')


    """    
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

            # mask
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
    async def close_coro(self):
        """
        if self._connected:
            try:
                self.writer.close()
                await self.writer.wait_closed()

                self.reader.close()

            except:
                pass
        """
        if self.writer is not None:
            self.writer.close()
            await self.writer.wait_closed()
        
        self._connected = False
        return True

    def close(self, timeout=5):
        """
        if self._connected:
            future = asyncio.run_coroutine_threadsafe(self.close_coro(), self.loop)
        """
        if self.loop is not None and self.loop.is_running():
            future = asyncio.run_coroutine_threadsafe(self.close_coro(), self.loop)
            
            # wait for timeout seconds
            try:
                return future.result(timeout=timeout)
            except asyncio.TimeoutError:
                # Handle the timeout error here
                return False
        return True

