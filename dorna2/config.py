config = {
    "server":{
        "ip": "localhost",
        "port": 443
    },
    "cmd_init":["alarm", "motor", "toollength", "input", "output", "pwm", "adc", "version", "uid", "tcp"],
    "T_camera_j4":[
        [5.25873615e-03, -9.99894519e-01, 1.34620306e-02, 4.65174596e+01],
        [9.99959617e-01, 5.35678348e-03, -7.35796480e-03, 3.20776662e+01],
        [7.28773209e-03, -1.35001806e-02, 9.99882310e-01, -4.24772615e+00],
        [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ],
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