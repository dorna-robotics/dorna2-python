import websocket
import json, threading, queue, time

class connection(object):

	def __init__(self):
		super(connection, self).__init__()
		self.sys = {}
		self.msg_q = queue.Queue()

	def connect_loop(self):
		def on_message(ws, msg):
			try:
				msg = json.loads(msg) # read the massage
				self.msg_q.put(msg) #put the message inside queue
				"""
				_sys = dict(self.sys) # make a copy
				for key in msg:
					_sys[key] = msg[key]
				self.sys = _sys # update sys
				"""
			except Exception as ex:
				print("error: ", ex)
				pass


		def on_error(ws, err):
			print("Error: ",err)


		def on_close(ws):
			print("Websocket is closed @"+self.url)


		def on_open(ws):
			print("Websocket is open @"+self.url)
			self.ws.connected = True

		self.ws = websocket.WebSocketApp(self.url,
									on_message = on_message,
									on_error = on_error,
									on_close = on_close,
									on_open = on_open)

		
		self.ws.connected = False
		self.ws.run_forever()


	def connect(self, url):
		print("connecting...")
		self.url = url	

		self.connect_thread = threading.Thread(target = self.connect_loop)
		self.connect_thread.start()
		
		while not self.ws.connected:
			time.sleep(0.001)


	def play(self, **arg):
		self.ws.send(json.dumps(arg))		
		


class dorna(connection):
	"""docstring for Dorna"""
	def __init__(self):
		super(dorna, self).__init__()


	def rand_id(self):
		return int(time.time()*10000)%1000000


	def joint(self):
	    return True


	def wait(self, _id):
		while True:
			try:
				if not self.msg_q.empty():
					msg = self.msg_q.get()
					if "stat" in msg and "id" in msg and msg["stat"] == 2 and msg["id"] == _id:
						break
			except Exception as ex:
				print(ex)
				pass


	def jmove(self,**arg):
		arg["cmd"] = "jmove"
		self.play(**arg)


	def lmove(self,**arg):
		arg["cmd"] = "lmove"
		self.play(**arg)
