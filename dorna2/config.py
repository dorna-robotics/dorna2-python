config = {
    "server":{
        "ip": "localhost",
        "port": 443
    },
    "cmd_init":["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid", "tcp"],
    "speed":{
        "very_quick":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 180, "accel": 2000, "jerk": 4000},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 100, "accel": 800, "jerk": 1000},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
        },
        "quick":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 0, "accel": 0, "jerk": 0},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 150, "accel": 2000, "jerk": 4000},
        },
        "moderate":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 0, "accel": 0, "jerk": 0},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 150, "accel": 2000, "jerk": 4000},
        },
        "slow":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 0, "accel": 0, "jerk": 0},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 150, "accel": 2000, "jerk": 4000},

        },
        "very_slow":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 0, "accel": 0, "jerk": 0},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 150, "accel": 2000, "jerk": 4000},
        },
        "custom":{
            "rmove":{"vel": 0, "accel": 0, "jerk": 0},
            "jmove":{"vel": 0, "accel": 0, "jerk": 0},
            "lmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cmove":{"vel": 0, "accel": 0, "jerk": 0},
            "clmove":{"vel": 0, "accel": 0, "jerk": 0},
            "cjmove":{"vel": 150, "accel": 2000, "jerk": 4000},
        }                                        
    },
    "log":{
        "path":"dorna.log",
    }     
}