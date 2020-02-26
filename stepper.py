from __future__ import division
from xml.dom import minidom

import RPi.GPIO as GPIO
import sys,time, threading


states_seq = [(1,0,1,0), (0,1,1,0), (0,1,0,1), (1,0,0,1)]
base_delay = 0.0055*2
event = threading.Event()

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class stepper():
    def __init__(self, pins, hysteresis=0):
        self.state = 0
        self.pins = pins
        self.last_dir = -1
        self.pos=0
        self.hysteresis = hysteresis
        for pin in pins:
            GPIO.setup(pin, GPIO.OUT)
        
    def set_pins(self, states):
        for pin,state in zip(self.pins, states):
            GPIO.output(pin, state)

    def _move(self, dir, steps, delay_factor):
        for _ in range(steps):
            self.state = (self.state+dir)%len(states_seq)
            self.set_pins(states_seq[self.state])
            time.sleep(delay_factor*base_delay)
                
    def _prep_move(self, dir):
        if dir !=0 and self.last_dir != dir:
            self._move(dir, self.hysteresis, 1)
            self.last_dir = dir

    def move(self, dir, steps, delay_factor):
        self._prep_move(dir)
        event.wait()
        self._move(dir, steps, delay_factor)
        self.pos+=(dir*steps)

rt_stepper = stepper((04, 17, 02, 03))
lt_stepper = stepper((05, 06, 13, 26))

def dir(x):
    if x==0: return 0
    return abs(x)//x

def delay(x, y):
    if x==0 or y==0: return (1,1)
    return max(y/x,1),max(x/y,1)

def move(x, y):
    event.clear()
    d = [dir(i) for i in (x,y)]
    m = [abs(i) for i in (x,y)]
    f = delay(*m)
    print(d,m,f)
    tx = threading.Thread(target=lt_stepper.move, args=(d[0], m[0], f[0]))
    ty = threading.Thread(target=rt_stepper.move, args=(d[1], m[1], f[1]))
    tx.start()
    ty.start()
    event.set()
    tx.join()
    ty.join()


def move_to(x,y):
    print((x-lt_stepper.pos, y-rt_stepper.pos))
    move(x-lt_stepper.pos, y-rt_stepper.pos)


#move(10,100)
#rt_stepper._move(-1,600,1)

doc = minidom.parse(sys.argv[1])
paths = [path.getAttribute('d') for path in doc.getElementsByTagName('path')]
for path in paths:
    path_parts = path.split(" ")
    start = path_parts[1]
    for path_part in path_parts[2:]:
        if "," in path_part:
            print(path_part)
            (x,y) = path_part.split(",")
            move_to(int(float(x)), int(float(y)))

GPIO.cleanup() 
