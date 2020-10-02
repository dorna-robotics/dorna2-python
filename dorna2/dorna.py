import websocket
import json, threading, queue, time

class connection(object):

	def __init__(self):
		super(connection, self).__init__()
		self.sys = {}
		self.q = queue.Queue()
		self.msg = queue.Queue(100)
		self.sys = {}


	def connect_loop(self):
		def on_message(ws, msg):
			try:
				msg = json.loads(msg) # read the massage
				
				#put the message inside queue
				self.q.put(msg)
				
			except Exception as ex:
				print("error: ", ex)
				pass


		def on_error(ws, err):
			print("Error: ",err)


		def on_close(ws):
			print("Websocket is closed @"+self.url)
			self.ws.connected = False


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


	def msg_loop(self):
		while self.ws.connected:
			try:
				if not self.q.empty():
					msg = self.q.get()
					self.sys = {**dict(self.sys), **msg} # update sys

					# put msg
					try:
						self.msg.put(msg, block = False)	
					except:
						pass

			except Exception as ex:
				print(ex)
				pass

	def connect(self, url, timeout = 20):
		print("connecting...")
		self.url = url	

		# connection thread
		self.connect_thread = threading.Thread(target = self.connect_loop)
		self.connect_thread.start()

		t = time.time()				
		while not self.ws.connected and time.time()-t <= timeout: 
			time.sleep(0.001)

		# message thread
		self.msg_thread = threading.Thread(target = self.msg_loop)
		self.msg_thread.start()


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


	def wait(self, _id, stop = False):
		while not stop:
			try:
				sys = dict(self.sys)
				if "stat" in sys and "id" in sys and sys["stat"] == 2 and sys["id"] == _id:
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
