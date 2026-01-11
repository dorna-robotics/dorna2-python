import importlib.resources 
import json
import random
import time
from dorna2.ws import WS
from dorna2.dof_5_6 import Kinematic
import logging
import logging.handlers
import copy
import numpy as np
import dorna2.pose as dorna_pose


class Dorna(WS):
    """docstring for Dorna"""
    def __init__(self, config=None, model="dorna_ta"):
        super(Dorna, self).__init__()

        if config is None:
            config = self.load_json("config.json")
        self.config = config
        
        # init logger
        self.logger = None

        # kinematic
        self.model = model    
        self.set_kinematic(model)



    def load_json(self, path, module_name="dorna2.cfg"):
        with importlib.resources.path(module_name, path) as config_file:
            with open(config_file, 'r') as file:
                return json.load(file)


    def dump_json(self, data, path, module_name="dorna2.cfg"):   
        with importlib.resources.path(module_name, path) as config_file:
            with open(config_file, 'w') as file:
                return json.dump(data, file, indent=4)


    def logger_setup(self, file="dorna.log", maxBytes=100000, backupCount=1):
        """Set up logging to log to rotating files and also console output."""
        formatter = logging.Formatter('%(asctime)s %(message)s')
        self.logger = logging.getLogger("dorna_log")
        self.logger.setLevel(logging.INFO)

        # rotating file handler
        fh = logging.handlers.RotatingFileHandler(file, maxBytes=maxBytes, backupCount=backupCount)
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)

        # console handler
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        self.logger.addHandler(ch)


    # last message received
    def recv(self):
        return copy.deepcopy(self._recv)

    
    def last_msg(self):
        return copy.deepcopy(self._recv)


    # last message sent
    def send(self):
        return copy.deepcopy(self._send)


    def last_cmd(self):
        return copy.deepcopy(self._send)


    def sys(self):
        return copy.deepcopy(self._sys)


    def union(self):
        return copy.deepcopy(self._sys)

    """
    return aggregate and all the messages in _msgs
    cmd, msgs, union
    """
    def track_cmd(self):
        # make a copy and rem
        _track = copy.deepcopy(self._track)
        del _track["id"]

        # create union
        union = {}
        for i in range(len(_track["msgs"])):
            union = {**union, **_track["msgs"][i]}
        return {** _track, "union": union}

    def format_numbers(self, obj):
        if isinstance(obj, dict):  # If the object is a dictionary
            return {key: self.format_numbers(value) for key, value in obj.items()}
        elif isinstance(obj, list):  # If the object is a list
            return [self.format_numbers(item) for item in obj]
        elif isinstance(obj, float):  # If the object is a float
            return round(obj, 3)  # Round to 3 decimal places
        else:
            return obj  # Return the object unchanged if it's neither dict, list, nor float


    def connect(self, host="localhost", port=443, timeout=5):
        # close the connection first
        if not self.close(timeout):
            return False 

        # Check the connection
        if not self.server(host, port, timeout):
            return False

        # initialize
        for cmd in self.config["cmd_init"]:
            self.cmd(cmd, timeout=1)

        return True


    def log(self, msg, *args, **kwargs):
        # setup log
        if self.logger == None:
            self.logger_setup()

        # print log
        self.logger.info(msg, *args, **kwargs)
    
    def rand_id(self, thr_low=100, thr_high= 1000000):
        return random.randint(thr_low, thr_high)


    """
    complete: wait for the completion or not
    timeout: the maximum amount of time (seconds), the API waits for the command completion, 
                negative (e.g. -1): wait for the command completion or error
                0: no waiting
                positive: wait for maximum timeout seconds to complete the command
    msg: command in form of text or dict
    **kwargs: Other parameters associated to the play
    --------
    return a dictionary formed by the union of the messages associated to this command 
    """
    def play(self, timeout=-1, msg=None, **kwargs):
        self._track = {"id": None, "msgs": [], "cmd": {}} # init track 

        # find msg
        if msg:
            # text
            if type(msg) == str:
                msg = json.loads(msg)

            elif type(msg) == dict:
                msg = copy.deepcopy(msg)
            else:
                return self.track_cmd()

        else:
            msg = copy.deepcopy(kwargs)

        # check the id
        if "id" in msg and type(msg["id"]) == int and msg["id"] > 0:
            pass
        else:
            msg["id"] = self.rand_id(100, 1000000)


        # remove all the None keys
        _msg = copy.deepcopy(msg)
        for key in _msg:
            if _msg[key] == None:
                del msg[key]

        # format
        #msg = self.format_numbers(msg)

        # set the tracking id
        self._track["cmd"] = copy.deepcopy(msg) 
        self._track["id"] =  msg["id"]
        
        # take a copy
        self._send = copy.deepcopy(msg)
        
        # write the message
        self.write(json.dumps(msg))

        start = time.time()
        while self._track["id"] == msg["id"]:            
            # positive timeout
            if timeout >=0 and time.time() >= start + timeout:
                break
            time.sleep(0.001)

        # finish tracking
        self._track["id"] = None
        return self.track_cmd()


    """
    # high level commands
    """

    """
    play a text file, where each line is a valid command, if a command is not valid, it will skip it and send the next one
    file: path to the script file
    timeout= if you want to wait for the completion of this block of scripts or not
        negative (-1): waits for its completion
        0: no wait
        positive: wait for maximum of timeout seconds
    --------
    return number valid commands that was sent  
    """
    def play_script(self, file="", timeout=-1):
        try:
            with open(file, 'r') as f:
                lines = f.readlines()
                for l in lines:
                    try:
                        self.play(timeout=0, msg=l)
                    except Exception as ex:
                        self.log(ex)
        except Exception as ex:
            self.log(ex)

        return self.sleep(0, timeout=timeout)


    # similar to play script but the commands are given as a list of dictionaries
    def play_list(self, cmd_list=[], timeout=-1):
        try:
            for cmd in cmd_list:
                try:
                    self.play(timeout=0, **cmd)
                except Exception as ex:
                    self.log(ex)
        except Exception as ex:
            self.log(ex)

        return self.sleep(0, timeout=timeout)


    def play_json(self, cmd='{}', timeout=-1):
        return self.play(timeout=timeout, msg=cmd)


    def play_dict(self, cmd={}, timeout=-1):
        return self.play(timeout=timeout, msg=cmd)


    """
    wait for a given patter in received signal
    """
    def wait(self, timeout=-1, **kwargs):
        # set wait dict
        self._ptrn["wait"] = copy.deepcopy(kwargs)
        # track
        if timeout >= 0:
            start = time.time()
            while time.time() <= start + timeout and self._ptrn["wait"]:
                time.sleep(0.001)
            self._ptrn["wait"] = None
        else:
            while self.ptrn:
                time.sleep(0.001)

        return copy.deepcopy(set( kwargs.items()) & set( self._ptrn["sys"].items()))

   
    """
    send a motion command
    """
    def _motion(self, method, **kwargs):
        cmd = {"cmd": method}
        
        if "pace" in kwargs:
            cmd = {**copy.deepcopy(cmd), **self.config["pace"][kwargs["pace"]][method]}
        cmd = {**copy.deepcopy(cmd), **kwargs}

        rtn = self.play(**cmd)

        # return stat
        try:
            return rtn["union"]["stat"]
        except:
            return False


    """
    return a dictionary based on keys
    if no *args is present it will return a copy of sys
    """        
    def get(self, *args):
        sys = self.union()
        return [sys[k] for k in args]


    """
    return one value based on the key
    """
    def val(self, key="cmd"):
        return self.union()[key]


    # It is a shorten version of play, 
    # set a parameter and wait for its reply from the controller
    def cmd(self, cmd, **kwargs):
        kwargs_clean = {k: v for k, v in kwargs.items() if v is not None}
        kwargs = {**{"cmd": cmd}, **kwargs_clean}
        return self.play(**kwargs)


    """
    send jmove, rmove, lmove, cmove command
    """
    def jmove(self, joint=[], rel=0, **kwargs):
        position = {f"j{i}": joint[i] for i in range(len(joint))}
        kwargs = {**position,"rel": rel, **kwargs}
        return self._motion("jmove", **kwargs)


    def rmove(self, **kwargs):
        return self._motion("rmove", **kwargs)        


    def lmove(self, pose=[], joint=[], rel=0, tool_pose=[0, 0, 0, 0, 0, 0], **kwargs):
        # set tool pose
        self.tool(tool=tool_pose)

        # set position
        if joint:
            position = {f"j{i}": joint[i] for i in range(len(joint))}
        elif pose:
            position = {["x", "y", "z", "a", "b", "c", "d", "e"][i]: pose[i] for i in range(len(pose))}

        kwargs = {**position,"rel": rel, **kwargs}
        return self._motion("lmove", **kwargs)


    def cmove(self, **kwargs):
        return self._motion("cmove", **kwargs)


    def _key_val_cmd(self, key, val, cmd, rtn_key, rtn_keys, **kwargs):
        # adjust key and kwargs
        if key != None and val != None:
            kwargs = {**{key:val}, **kwargs}

        # send
        rtn = self.cmd(cmd, **kwargs)
        
        # return
        try:
            if rtn_key:
                return rtn["union"][rtn_key]
            else:
                return [rtn["union"][k] for k in rtn_keys]
        except:
            return False

  
    def _track_cmd_stat(self):
        rtn = self.track_cmd()
        try:
            return rtn["union"]["stat"]
        except:
            return False


    """
    read output i, or turn it on or off
    output(0): return the value of the out0
    output(0,1): set the value of the out0 to 1
    output(): return the value of the outputs
    output(out0=1, out2=0): set the value of out0 to 1 and out2 to 0 

    return -> the value of one out or all outs
    """
    def output(self, index=None, val=None, config=None, **kwargs):
        if config is not None:
            cmd_list = []   
            for c in config:
                if len(c) >= 2 and c[0] in range(16) and c[1] in range(2):
                    cmd_list.append({"cmd": "output", "out" + str(c[0]): c[1], "queue": 0})
                    if len(c) > 2 and c[2] > 0:
                        cmd_list.append({"cmd": "sleep", "time": c[2], "queue": 0})
            return self.play_list(cmd_list)

        key = None
        if index !=None:
            key = "out"+str(index)
        
        cmd = "output"
        rtn_key = key
        rtn_keys = ["out"+str(i) for i in range(16)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)



    def get_all_output(self, **kwargs):
        return self.output(**kwargs)


    def get_output(self, index=None, **kwargs):
        return self.output(index=index, **kwargs)


    def set_output(self, index=None, val=None, queue=None, **kwargs):
        self.output(index=index, val=val, queue=queue, **kwargs)
        return self._track_cmd_stat()


    """
    read pwm, or set its parameters
    pwm(0): return the value of the pwm0
    pwm(0,1): set the value of the pwm0 to 1
    pwm(): return the value of the pwms
    pwm(pwm0=1, freq2=1): set the value of pwm0 to 1 and freq2 to 1

    return the value of pwm or all of them    
    """
    def pwm(self, index=None, val=None, **kwargs):
        key = None
        if index !=None:
            key = "pwm"+str(index)
        
        cmd = "pwm"
        rtn_key = key
        rtn_keys = ["pwm"+str(i) for i in range(5)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    """
    read freq, or set its parameters
    freq(0): return the value of the freq0
    freq(0,1): set the value of the freq0 to 1
    """
    def freq(self, index=None, val=None, **kwargs):
        key = None
        if index !=None:
            key = "freq"+str(index)
        
        cmd = "pwm"
        rtn_key = key
        rtn_keys = ["freq"+str(i) for i in range(5)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    """
    read duty, or set its parameters
    duty(0): return the value of the duty0
    duty(0,1): set the value of the duty0 to 1
    """
    def duty(self, index, val=None, **kwargs):
        key = None
        if index !=None:
            key = "duty"+str(index)
        
        cmd = "pwm"
        rtn_key = key
        rtn_keys = ["duty"+str(i) for i in range(5)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_pwm(self, index=None, **kwargs):
        return self.pwm(index=index, **kwargs)


    def get_freq(self, index=None, **kwargs):
        return self.freq(index=index, **kwargs)


    def get_duty(self, index=None, **kwargs):
        return self.duty(index=index, **kwargs) 


    def set_pwm(self, index=None, enable=None, queue=None, **kwargs):
        self.pwm(index=index, val=enable, queue=queue, **kwargs)
        return self._track_cmd_stat()


    def set_freq(self, index=None, freq=None, queue=None, **kwargs):
        self.freq(index=index, freq=freq, queue=queue, **kwargs)
        return self._track_cmd_stat()


    def set_duty(self, index=None, duty=None, queue=None, **kwargs):
        self.duty(index=index, duty=index, queue=queue, **kwargs)
        return self._track_cmd_stat()


    """
    read input
    """
    def input(self, index=None, **kwargs):
        key = None
        if index !=None:
            key = "in"+str(index)
        
        cmd = "input"
        rtn_key = key
        rtn_keys = ["in"+str(i) for i in range(16)]
        val = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_all_input(self, **kwargs):
        return self.input(**kwargs)


    def get_input(self, index=None, **kwargs):
        return self.input(index=index, **kwargs)


    """
    read adc
    """
    def adc(self, index=None, **kwargs):
        key = None
        if index !=None:
            key = "adc"+str(index)
        
        cmd = "adc"
        rtn_key = key
        rtn_keys = ["adc"+str(i) for i in range(5)]
        val = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)

    def get_all_adc(self, **kwargs):
        return self.adc(**kwargs)

    def get_adc(self, index=None, **kwargs):
        return self.adc(index=index, **kwargs)

 
    """
    probe
    """
    def probe(self, index=None, val=None, **kwargs):
        key = None
        if index !=None:
            key = "in"+str(index)
        
        cmd = "probe"
        rtn_key = None
        rtn_keys = ["j"+str(i) for i in range(8)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    """
    iprobe
    return joint values in a list
    """
    def iprobe(self, index=None, val=None, **kwargs):
        key = None
        if index !=None:
            key = "in"+str(index)
        
        cmd = "iprobe"
        rtn_key = None
        rtn_keys = ["j"+str(i) for i in range(8)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)

    
    """
    send a halt command
    """
    def halt(self, accel=None, **kwargs):
        key = "accel"
        val = accel
        cmd = "halt"
        rtn_key = "stat"
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    """
    read alarm status, set or unset alarm
    """
    def alarm(self, val=None, **kwargs):
        key = "alarm"
        cmd = "alarm"
        rtn_key = key
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_alarm(self, **kwargs):
        return self.alarm(**kwargs)


    def set_alarm(self, enable=None, **kwargs):
        self.alarm(val=enable, **kwargs)
        return self._track_cmd_stat()


    """
    sleep the controller for certain amount of time (seconds)
    """
    def sleep(self, val=None, **kwargs):
        key = "time"
        cmd = "sleep"
        rtn_key = "stat"
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)

   
    """
    set the value of joints / and return their values in a dictionary

    [list]: return the joint values in a list format
    """
    def joint(self, index=None, val=None, **kwargs):
        # read the joints
        if not val and not kwargs:
            try:
                return self.val("j"+str(index))
            except:
                return [self.val("j"+str(k)) for k in range(8)]  

        key = None
        if index !=None:
            key = "j"+str(index)
        
        cmd = "joint"
        rtn_key = key
        rtn_keys = ["j"+str(i) for i in range(8)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_all_joint(self):
        return self.joint()


    def get_joint(self, index=None):
        return self.joint(index=index)


    def set_joint(self, index=None, val=None, **kwargs):
        self.joint(index=index, val=val, **kwargs)
        return self._track_cmd_stat()


    def pose(self, index=None):
        """
        Get the robot x, y, z, a, b, c, d and e poses. 

        Returns:
            (list of length 8): The robot pose.
        """
        _pose = self.get("x", "y", "z", "a", "b", "c", "d", "e")
        try:
            return _pose[index]
        except:
            return _pose


    def get_all_pose(self):
        return self.pose()    


    def get_pose(self, index=None):
        return self.pose(index=index)


    def motor(self, val=None, **kwargs):
        """
        Enable or disable the robot motors. Or get the motor status (disabled or enabled).
        If the val parameter is not specified, then we get the motor status. 

        Parameters:
            val (int): Use this parameter to enable (val=1) or disable (val=1) the motors. 

        Returns:
            (int): The status of the motors.
        """
        key = "motor"
        cmd = "motor"
        rtn_key = key
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_motor(self, **kwargs):
        return self.motor(**kwargs)


    def set_motor(self, enable=None, **kwargs):
        self.motor(val=enable, **kwargs)
        return self._track_cmd_stat()


    def toollength(self, val=None, **kwargs):
        """
        Set and get the robot tool length. 
        If the length of the toollength is not specified then, we get the value of the toollength.

        Parameters:
            val (float or None): The length of the toollength in mm.

        Returns:
            (float): The robot toollength in mm.
        """
        key = "toollength"
        cmd = "toollength"
        rtn_key = key
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_toollength(self, **kwargs):
        return self.toollength(**kwargs)


    def set_toollength(self, length=None ,**kwargs):
        self.toollength(val=length, **kwargs)
        return self._track_cmd_stat()


    def tool(self,tool=[0, 0, 0, 0, 0, 0], mtrx=None, **kwargs):
        if tool is not None and mtrx is None:
            mtrx = dorna_pose.xyzabc_to_T(tool)
        cmd = {"cmd": "tool", "r00": mtrx[0][0], "r01": mtrx[0][1], "r02": mtrx[0][2], "r10": mtrx[1][0], "r11": mtrx[1][1], "r12": mtrx[1][2], "r20": mtrx[2][0], "r21": mtrx[2][1], "r22": mtrx[2][2],
                "lx": mtrx[0][3], "ly": mtrx[1][3], "lz": mtrx[2][3]}
        return self.play(**cmd)

    def version(self, **kwargs):
        """
        Get the version of the firmware running on the controller.

        Parameters:

        Returns:
            (int): The version of the firmware.
        """
        key = None
        val = None
        cmd = "version"
        rtn_key = "version"
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def uid(self, **kwargs):
        """
        Get the controller UID number.

        Parameters:

        Returns:
            (str): The uid of the controller.
        """
        key = None
        val = None
        cmd = "uid"
        rtn_key = "uid"
        rtn_keys = None

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def gravity(self, **kwargs):
        key = None
        val = None        
        cmd = "gravity"
        rtn_key = key
        rtn_keys = ["gravity", "m", "x", "y", "z"]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_gravity(self, **kwargs):
        return self.gravity(**kwargs)


    def set_gravity(self, enable=None, mass=None, x=None, y=None, z=None, **kwargs):
        self.gravity(gravity=enable, m=mass, x=x, y=y, z=z, **kwargs)
        return self._track_cmd_stat()


    def axis(self, index=None, val=None, **kwargs):
        key = None
        if index !=None:
            key = "ratio"+str(int(index))
        
        cmd = "axis"
        rtn_key = key
        rtn_keys = ["ratio"+str(i) for i in range(5,8)]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def get_axis(self, index=None, **kwargs):
        return self.axis(index=index, **kwargs)


    def set_axis(self, index=None, ratio=None, **kwargs):
        self.axis(index=index, val=ratio, **kwargs)
        return self._track_cmd_stat()


    def get_axis_ratio(self, index=None, **kwargs):
        return self.axis(index=index, **kwargs)


    def set_axis_ratio(self, index=None, ratio=None,  **kwargs):
        self.axis(index=index, val=ratio, **kwargs)
        return self._track_cmd_stat()


    def get_axis(self, index=None, **kwargs):
        return_keys = [key+str(int(index)) for key in ["usem", "usee", "pprm", "tprm", "ppre", "tpre"]]
        return self._key_val_cmd(None, None, "axis", None, return_keys, **kwargs)


    def set_axis(self, index=None, usem=None, usee=None, pprm=None, tprm=None, ppre=None, tpre=None, **kwargs):
        return_keys = [key+str(int(index)) for key in ["usem", "usee", "pprm", "tprm", "ppre", "tpre"]]
        key_value = {k: v for k, v in zip(return_keys, [usem, usee, pprm, tprm, ppre, tpre])}
        self._key_val_cmd(None, None, "axis", None, return_keys, **key_value, **kwargs)
        return self._track_cmd_stat()


    def get_emergency(self):
        return dict(self._emergency)


    def set_emergency(self, enable=False, key="in0", value=1):
        self._emergency = {"enable":enable, "key":key, "value":value}
        return self.get_emergency()

    """
    def pid(self, index, **kwargs):
        key = None
        val = None        
        cmd = "pid"
        rtn_key = None
        rtn_keys = [prm+str(int(index)) for prm in ["p", "i", "d", "threshold", "duration"]]

        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)
    """

    def get_pid(self, index=None, **kwargs):
        key = None
        val = None        
        cmd = "pid"
        rtn_key = None
        rtn_keys = [prm+str(int(index)) for prm in ["p", "i", "d", "threshold", "duration"]]
        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)


    def set_pid(self, index=None, p=None, i=None, d=None, threshold=None, duration=None, **kwargs):
        self.get_pid(index=index, **{"p"+str(int(index)): p, "i"+str(int(index)): i, "d"+str(int(index)): d, "threshold"+str(int(index)): threshold, "duration"+str(int(index)): duration})
        return self._track_cmd_stat()


    def get_pid_enable(self, **kwargs):
        key = "pid"
        val = None        
        cmd = "pid"
        rtn_key = "pid"
        rtn_keys = None
        return self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)

            
    def set_pid_enable(self, enable=None, **kwargs):
        key = "pid"
        val = enable        
        cmd = "pid"
        rtn_key = "pid"
        rtn_keys = None
        self._key_val_cmd(key, val, cmd, rtn_key, rtn_keys, **kwargs)
        return self._track_cmd_stat()
    
    def pid_enable(self, **kwargs):
        return self.set_pid_enable(enable=True, **kwargs)


    def pid_disable(self, **kwargs):
        return self.set_pid_enable(enable=False, **kwargs)


    def set_kinematic(self, model="dorna_ta", **kwargs):
        self.kinematic = Kinematic(model, self.config["limit"][model])


    def go(self, pose=None, joint=None, ej=[0, 0, 0, 0, 0, 0, 0, 0], frame=[0, 0, 0, 0, 0, 0], tool=[0, 0, 0, 0, 0, 0], cmd_list=[], current_joint=None, motion="jmove", vaj=None, speed=0.2, freedom={"num":200, "range":[2, 2, 2], "early_exit":True}, cont=0, corner=50, timeout=-1, sim=0,  **kwargs):        

        if joint is None and pose is not None:
            # Kinematic
            self.kinematic.set_tcp_xyzabc(tool)

            # Current
            current_joint = current_joint if current_joint is not None else self.get_all_joint()[0:6]
            current_pose = self.kinematic.fw(joint=current_joint)

            # pose and joint
            if len(pose) == 3:
                pose = [x for x in pose]+current_pose[3:]
            
            joint = self.kinematic.inv(pose[0:6], current_joint[0:6], False, freedom=freedom)[0]

            # add auxiliary axes
            joint = np.append(joint, pose[6:])

        # copy
        joint = np.array(joint)
        
        # ej
        for i in range(min(len(ej), len(joint))):
            joint[i] -= ej[i]
 
        # Speed
        speed = min(1, max(0, speed))

        # vaj
        if vaj is None:
            vaj = [x * speed for x in self.config["speed"]["very_quick"][motion].values()]
        
        # cmds
        cmd_motion = {"cmd": motion, "rel": 0, "vel": vaj[0], "accel": vaj[1], "jerk": vaj[2], "cont": cont, "corner": corner}
        for i in range(len(joint)):
            cmd_motion["j"+str(i)] = float(joint[i])
        _cmd_list = [
            cmd_motion,
            *cmd_list,
        ]

        # sim
        if not sim:
            # play        
            for cmd in _cmd_list:
                self.play(timeout=timeout, msg=cmd)
        
        self.kinematic.set_tcp_xyzabc([0, 0, 0, 0, 0, 0])
        return _cmd_list

    
    def pick_n_place(self,
                    pick={},
                    place={},
                    middle=[],
                    end={},
                    base_in_world=[0, 0, 0, 0, 0, 0],
                    aux_dir=[[1, 0, 0], [0, 0, 0]],
                    sleep=0.5,
                    current_joint=None, motion="jmove", vaj=None, cvaj=None, speed=0.2, cont=1, corner=100, 
                    freedom={"num":200, "range":[2, 2, 2], "early_exit":True }, timeout=-1, sim=0, **kwargs,
        ):
        
        # location
        _location = {
            "type": "robot", # robot, world, joint
            "loc": None, # [[x, y, z, a, b, c], [aux0, aux1]]
            "tool": [None, None],
            "frame": [0, 0, 0, 0, 0, 0],
            "ej": [0, 0, 0, 0, 0, 0, 0, 0],
            "output": [],
            "cmd": [],
            "approach": [],
            "exit": [],
            }

        try:

            pick = {**_location, **pick}
            place = {**_location, **place}
            middle = [{**_location, **m} for m in middle]
            end = {**_location, **end}

            # init
            cmd_list = []
                    
            # Speed
            speed = min(1, max(0, speed))

            # vaj
            if vaj is None:
                vaj = [x * speed for x in self.config["speed"]["very_quick"][motion].values()]
            
            # cvaj
            if cvaj is None:
                if cont == 1:
                    cvaj = [x * speed for x in self.config["speed"]["very_quick"]["c"+motion].values()]
                else:
                    cvaj = vaj

            ###########################
            ###### current joint ######
            ###########################
            self.kinematic.set_tcp_xyzabc(pick["tool"][0])
            current_joint = current_joint if current_joint is not None else self.get_all_joint()
            current_joint = (current_joint + [0, 0, 0, 0, 0, 0, 0, 0])[:8]
            current_pose = np.array(self.kinematic.fw(joint=current_joint[0:6]))
            current_aux = current_joint[6:8]

            # last joint
            last_joint = np.array(current_joint)


            ##################
            ###### Pick ######
            ##################
            # pose
            ignore_pick = False
            if pick["loc"] is None:
                ignore_pick = True
                pick_pose = np.array(current_pose)
                pick_aux = current_aux
            elif pick["type"] == "robot":
                pick_pose = np.array(dorna_pose.transform_pose(pick["loc"][0], from_frame=pick["frame"]))
                pick_aux = pick["loc"][1]
            elif pick["type"] == "world":
                pick_pose = np.array(dorna_pose.frame_to_robot(pick["loc"][0], pick["loc"][1], aux_dir=aux_dir, frame_in_world=pick["frame"], base_in_world=base_in_world))      
                pick_aux = pick["loc"][1]
            elif pick["type"] == "joint":
                self.kinematic.set_tcp_xyzabc(pick["tool"][0])
                pick_pose = np.array(self.kinematic.fw(joint=pick["loc"][0][0:6]))
                pick_aux = pick["loc"][1]
            # approach pick pose
            approach_pick_pose_list = [dorna_pose.transform_pose(pose, from_frame=pick_pose) for pose in pick["approach"]]
            
            # exit pick pose
            exit_pick_pose_list = [dorna_pose.transform_pose(pose, from_frame=pick_pose) for pose in pick["exit"]]

            # approach pick joint
            approach_pick_joint_list = []
            self.kinematic.set_tcp_xyzabc(pick["tool"][0])
            for pose in approach_pick_pose_list:
                last_joint = np.append(self.kinematic.inv(pose, last_joint[0:6], False, freedom=freedom)[0], pick_aux)
                approach_pick_joint_list.append(last_joint)

            # pick joint
            pick_joint = np.append(self.kinematic.inv(pick_pose, last_joint[0:6], False, freedom=freedom)[0], pick_aux)
            last_joint = np.array(pick_joint) # update last joint

            # exit pick joint
            exit_pick_joint_list = []
            self.kinematic.set_tcp_xyzabc(pick["tool"][1])
            for pose in exit_pick_pose_list:
                last_joint = np.append(self.kinematic.inv(pose, last_joint[0:6], False, freedom=freedom)[0], pick_aux)
                exit_pick_joint_list.append(last_joint)

            # ej
            for joint in approach_pick_joint_list + [pick_joint] + exit_pick_joint_list:
                for i in range(min(len(pick["ej"]), len(joint))):
                    joint[i] -= pick["ej"][i]
                    
            # cmd pick approach
            if not ignore_pick:
                for joint in approach_pick_joint_list + [pick_joint]:
                    cmd_list.append(
                        {"cmd": motion, "rel": 0, "vel": vaj[0], "accel": vaj[1], "jerk": vaj[2], "cont": 0} | {"j"+str(i): float(joint[i]) for i in range(len(joint))}
                    )

            # cmd pick output
            for output_config in pick["output"]:
                cmd_list.append({"cmd": "output", "out" + str(output_config[0]): output_config[1], "queue": 0})
                if len(output_config) > 2 and output_config[2] > 0:
                    cmd_list.append({"cmd": "sleep", "time": output_config[2], "queue": 0})

            # cmd pick_sleep
            cmd_list.append({"cmd": "sleep", "time": sleep})

            # pick cmd list
            cmd_list += pick["cmd"]

            # cmd pick exit
            for joint in exit_pick_joint_list:
                cmd_list.append(
                    {"cmd": motion, "rel": 0, "vel": cvaj[0], "accel": cvaj[1], "jerk": cvaj[2], "cont": cont, "corner": corner} | {"j"+str(i): float(joint[i]) for i in range(len(joint))}
                )


            ##################
            ###### place #####
            ##################
            place_pose = None
            if place["loc"] is not None:
                # tool
                if place["type"] == "robot":
                    place_pose = np.array(dorna_pose.transform_pose(place["loc"][0], from_frame=place["frame"]))
                    place_aux = place["loc"][1]
                elif place["type"] == "world":
                    place_pose = np.array(dorna_pose.frame_to_robot(place["loc"][0], place["loc"][1], aux_dir=aux_dir, frame_in_world=place["frame"], base_in_world=base_in_world))      
                    place_aux = place["loc"][1]
                elif place["type"] == "joint":
                    self.kinematic.set_tcp_xyzabc(place["tool"][0])
                    place_pose = np.array(self.kinematic.fw(joint=place["loc"][0][0:6]))
                    place_aux = place["loc"][1]

                # approach place pose
                approach_place_pose_list = [dorna_pose.transform_pose(pose, from_frame=place_pose) for pose in place["approach"]]

                # exit place pose
                exit_place_pose_list = [dorna_pose.transform_pose(pose, from_frame=place_pose) for pose in place["exit"]]


            ##################
            ###### middle ####
            ##################
            middle_joint = []
            for mp in middle:
                if mp["loc"] is None:
                    continue

                if mp["type"] == "joint":
                    mp_joint = mp["loc"][0] + mp["loc"][1]
                    mp_aux = list(mp["loc"][1])
                else:
                    if mp["type"] == "robot":
                        mp_pose = np.array(dorna_pose.transform_pose(mp["loc"][0], from_frame=place["frame"]))
                        mp_aux = list(mp["loc"][1])
                    elif mp["type"] == "world":
                        mp_pose = np.array(dorna_pose.frame_to_robot(mp["loc"][0], mp["loc"][1], aux_dir=aux_dir, frame_in_world=mp["frame"], base_in_world=base_in_world))      
                        mp_aux = list(mp["loc"][1])
                    
                    self.kinematic.set_tcp_xyzabc(mp["tool"])
                    mp_joint = np.append(self.kinematic.inv(mp_pose, last_joint[0:6], False, freedom=freedom)[0], mp_aux)
                
                # cmd
                middle_joint.append(mp_joint)
                last_joint = list(mp_joint)
                cmd_list += [{"cmd": motion, "rel": 0} | {"j"+str(i): float(mp_joint[i]) for i in range(len(mp_joint))}]

            #####################
            ###### place again ##
            #####################
            if place_pose is not None:
                # approach
                approach_place_joint_list = []
                self.kinematic.set_tcp_xyzabc(place["tool"][0])
                for pose in approach_place_pose_list:
                    last_joint = np.append(self.kinematic.inv(pose, last_joint[0:6], False, freedom=freedom)[0], place_aux)
                    approach_place_joint_list.append(last_joint)
                
                place_joint = np.append(self.kinematic.inv(place_pose[0:6], last_joint[0:6], False, freedom=freedom)[0], place_aux)
                last_joint = np.array(place_joint) # update last joint

                # exit
                exit_place_joint_list = []
                self.kinematic.set_tcp_xyzabc(place["tool"][1])
                for pose in exit_place_pose_list:
                    last_joint = np.append(self.kinematic.inv(pose, last_joint[0:6], False, freedom=freedom)[0], place_aux)
                    exit_place_joint_list.append(last_joint)


                # ej
                for joint in approach_place_joint_list + [place_joint] + exit_place_joint_list:
                    for i in range(min(len(place["ej"]), len(joint))):
                        joint[i] -= place["ej"][i]

                
                # cmd
                for joint in approach_place_joint_list:
                    cmd_list.append(
                        {"cmd": motion, "rel": 0, "cont": 0} | {"j"+str(i): float(joint[i]) for i in range(len(joint))}
                    )
                cmd_list.append(
                    {"cmd": motion, "rel": 0, "vel": vaj[0], "accel": vaj[1], "jerk": vaj[2]} | {"j"+str(i): float(place_joint[i]) for i in range(len(place_joint))}
                )

                for output_config in place["output"]:
                    cmd_list.append({"cmd": "output", "out" + str(output_config[0]): output_config[1], "queue": 0})
                    if len(output_config) > 2 and output_config[2] > 0:
                        cmd_list.append({"cmd": "sleep", "time": output_config[2], "queue": 0})



                cmd_list += [{"cmd": "sleep", "time": sleep},
                    *place["cmd"]]
                
                # exit place
                for joint in exit_place_joint_list:
                    cmd_list.append(
                        {"cmd": motion, "rel": 0, "vel": cvaj[0], "accel": cvaj[1], "jerk": cvaj[2], "cont": cont} | {"j"+str(i): float(joint[i]) for i in range(len(joint))}
                    )


            ###############
            ###### end ####
            ###############
            end_joint = None
            if end["loc"] is not None:
                if end["type"] == "joint":
                    end_joint = end["loc"][0] + end["loc"][1]
                    end_aux = end["loc"][1]
                else:
                    if end["type"] == "robot":
                        end_pose = np.array(dorna_pose.transform_pose(end["loc"][0], from_frame=end["frame"]))
                        end_aux = end["loc"][1]
                    elif end["type"] == "world":
                        end_pose = np.array(dorna_pose.frame_to_robot(end["loc"][0], end["loc"][1], aux_dir=aux_dir, frame_in_world=end["frame"], base_in_world=base_in_world))
                        end_aux = end["loc"][1]
                    
                    self.kinematic.set_tcp_xyzabc(end["tool"])
                    end_joint = np.append(self.kinematic.inv(end_pose, last_joint[0:6], False, freedom=freedom)[0], end_aux)
                
            # end joint
            if end_joint is not None:
                cmd_list +=[
                    {"cmd": motion, "rel": 0} | {"j"+str(i): float(end_joint[i]) for i in range(len(end_joint))}
                ]

            # sim
            if sim:
                return cmd_list
            
            # play list
            return self.play_list(cmd_list, timeout=timeout)

        except Exception as ex:
            self.log(ex)
        return None


    """
    home the robot with the encoder index
    index: joint index 0-7
    val: value assigned to the joint after homing
    probe_val: value to trigger the index pulse
    dir: -1=negative, 1=positive
    travel: travel for homing
    vel: velocity for homing
    accel: acceleration for homing
    jerk: jerk for homing
    timeout: -1=infinite, >0 timeout in seconds
    return: status
    """
    def home_with_encoder_index(self, index=7, val=0, probe_val=1, dir=1, travel=1000, vel=100, accel=1000,jerk=5000, timeout=60, **kwargs):
        # jmove
        cmd_jmove = {"j"+str(index): travel*dir, "rel": 1, "vel": vel, "accel": accel, "jerk": jerk, "cont":0}
        self.jmove(**cmd_jmove, timeout=0)

        # run iprobe
        result = self.iprobe(index=index, val=probe_val, timeout=timeout)

        # halt
        self.halt()

        # assign the value to the joint
        if result:
            # sleep
            time.sleep(0.1)

            # offset: current joint - probed value
            offset = self.get_joint(index=index) - result[index]

            # set joint
            return self.set_joint(index=index, val=val + offset)
        
        return None



