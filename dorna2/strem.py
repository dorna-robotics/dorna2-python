import asyncio
import threading
import json
import time

class comm(object):
    """docstring for comm"""
    def __init__(self):
        super(comm, self).__init__()

    async def _connect(self, ip, port, msg, msg_cmd):
        # connect
        self.reader, self.writer = await asyncio.open_connection(ip, port)
        
        # handshake
        self.writer.write(self.process_write(mode="handshake"))
        await self.writer.drain()

        await asyncio.sleep(1)
        self.connected = True
        self.flag = True

        s = time.time()
        # read task
        await asyncio.gather(self.write(msg_cmd), self.read())
        print(time.time()-s)


    async def write(self, msg_cmd):
        while self.connected:            
            if self.flag:
                try:
                    msg = msg_cmd.pop(0)
                    msg = self.process_write(json.dumps(msg))
                    self.writer.write(msg)
                    await self.writer.drain()
                    self.flag = False
                except Exception as ex:
                    print(ex)
            else:
                await asyncio.sleep(0)

    async def read(self):
        state = 0
        buff = b""
        waiting_len = 1

        while self.connected:
            data = await self.reader.read(100)

            if len(data):                
                # add to the buffer
                buff += data
                # process buffer
                buff, waiting_len, state = self.process_buffer(buff, waiting_len, state)
                #print(time.time())


    def process_write(self, msg="", mode="cmd"):
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
                    #print(msg)
                    """
                    """
                    if "stat" in msg and msg["stat"] == 2:
                        print(msg)
                        if "id" in msg and msg["id"] == 10000000:
                            self.connected = False
                        else:
                            self.flag = True
                    """
                    """ 
                    """
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
                    """
                except Exception as ex:
                    print(ex)
                    pass

                state = 0
                buff = buff[waiting_len:]
                waiting_len = 1

        return buff, waiting_len, state  

    async def _close():
        self.writer.close()
        await self.writer.wait_closed()


async def main(msg):
    reader, writer = await asyncio.open_connection(
        '192.168.1.8', 443)

    writer.write(msg)
    await writer.drain()
    
    while True:
        data = await reader.read(100)
        if len(data):
            print(data)
    
    writer.close()
    await writer.wait_closed()

if __name__ == '__main__':
    ip = "192.168.254.77"
    port = 443
    
    msg_cmd = [{"cmd": "alarm", "id": 247811}, {"cmd": "motor", "id": 543268}, {"cmd": "toollength", "id": 479913}, {"cmd": "input", "id": 250854}, {"cmd": "output", "id": 123840}, {"cmd": "pwm", "id": 966262}, {"cmd": "adc", "id": 366636}, {"cmd": "version", "id": 524450}, {"cmd": "uid", "id": 449356}, {"cmd": "alarm", "id": 288300}, {"cmd": "motor", "id": 36271}, {"cmd": "toollength", "id": 538962}, {"cmd": "input", "id": 33249}, {"cmd": "output", "id": 772856}, {"cmd": "pwm", "id": 440853}, {"cmd": "adc", "id": 466343}, {"cmd": "version", "id": 461662}, {"cmd": "uid", "id": 856698}, {"cmd": "alarm", "id": 566755}, {"cmd": "motor", "id": 881235}, {"cmd": "toollength", "id": 747494}, {"cmd": "input", "id": 416981}, {"cmd": "output", "id": 359105}, {"cmd": "pwm", "id": 276240}, {"cmd": "adc", "id": 644873}, {"cmd": "version", "id": 513655}, {"cmd": "uid", "id": 633871}, {"cmd": "alarm", "id": 327526}, {"cmd": "motor", "id": 988049}, {"cmd": "toollength", "id": 714100}, {"cmd": "input", "id": 877750}, {"cmd": "output", "id": 903162}, {"cmd": "pwm", "id": 720398}, {"cmd": "adc", "id": 95881}, {"cmd": "version", "id": 230524}, {"cmd": "uid", "id": 339134}, {"cmd": "alarm", "id": 824731}, {"cmd": "motor", "id": 559853}, {"cmd": "toollength", "id": 119585}, {"cmd": "input", "id": 261408}, {"cmd": "output", "id": 466726}, {"cmd": "pwm", "id": 42384}, {"cmd": "adc", "id": 375620}, {"cmd": "version", "id": 657682}, {"cmd": "uid", "id": 81825}, {"cmd": "alarm", "id": 420627}, {"cmd": "motor", "id": 636342}, {"cmd": "toollength", "id": 106869}, {"cmd": "input", "id": 383229}, {"cmd": "output", "id": 659064}, {"cmd": "pwm", "id": 889376}, {"cmd": "adc", "id": 80351}, {"cmd": "version", "id": 737340}, {"cmd": "uid", "id": 746011}, {"cmd": "alarm", "id": 386880}, {"cmd": "motor", "id": 476258}, {"cmd": "toollength", "id": 399150}, {"cmd": "input", "id": 743022}, {"cmd": "output", "id": 202337}, {"cmd": "pwm", "id": 542573}, {"cmd": "adc", "id": 607797}, {"cmd": "version", "id": 971344}, {"cmd": "uid", "id": 273586}, {"cmd": "alarm", "id": 968605}, {"cmd": "motor", "id": 315570}, {"cmd": "toollength", "id": 841496}, {"cmd": "input", "id": 666908}, {"cmd": "output", "id": 238716}, {"cmd": "pwm", "id": 198602}, {"cmd": "adc", "id": 170835}, {"cmd": "version", "id": 563931}, {"cmd": "uid", "id": 628681}, {"cmd": "alarm", "id": 59904}, {"cmd": "motor", "id": 969653}, {"cmd": "toollength", "id": 898689}, {"cmd": "input", "id": 726837}, {"cmd": "output", "id": 385607}, {"cmd": "pwm", "id": 582708}, {"cmd": "adc", "id": 38332}, {"cmd": "version", "id": 168416}, {"cmd": "uid", "id": 520255}, {"cmd": "alarm", "id": 742095}, {"cmd": "motor", "id": 625252}, {"cmd": "toollength", "id": 142922}, {"cmd": "input", "id": 978231}, {"cmd": "output", "id": 121801}, {"cmd": "pwm", "id": 128300}, {"cmd": "adc", "id": 107804}, {"cmd": "version", "id": 907098}, {"cmd": "uid", "id": 1000}]
    msg = (
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
    

    communication = comm()
    #asyncio.run(communication._connect(ip, port, msg, msg_cmd))
    _thread = threading.Thread(target=asyncio.run, args=(communication._connect(ip, port, msg, 1000* msg_cmd),))
    _thread.start()    



