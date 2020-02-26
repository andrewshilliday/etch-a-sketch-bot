from __future__ import division
from xml.dom import minidom

import RPi.GPIO as GPIO
import sys,time,re,io, math

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

phases = [(1,0,1,0), (0,1,1,0), (0,1,0,1), (1,0,0,1)]


class Stepper():
    idx=0
    dir=0
    pos=0

    def __init__(self, a1, a2, b1, b2):
        self.idx=0
        self.dir=0
        self.pos=0
        self.pins=[a1, a2, b1, b2]

        for pin in self.pins:
            GPIO.setup(pin, GPIO.OUT)


    def move(self, dir, steps, delay=0.002):
        for _ in range(steps):
            next_idx=(self.idx + dir)%len(phases)

            for (pin,state) in zip(self.pins, phases[next_idx]):
                GPIO.output(pin, state)

            self.idx = next_idx
            self.dir = dir
            self.pos += dir

            time.sleep(delay)

    def unhold(self):
        for pin in self.pins:
            GPIO.output(pin, 0)
            

class Control:
    ystepper = Stepper(05, 06, 13, 26)
    xstepper = Stepper(04, 17, 02, 03) #, 04, 17)
    
    def line(self, newx, newy):
        print(newx, newy, newx - self.xstepper.pos, newy - self.ystepper.pos)

        dx = newx - self.xstepper.pos
        dy = newy - self.ystepper.pos

        dirx = 1 if dx > 0 else -1
        diry = 1 if dy > 0 else -1

        dx = int(abs(dx))
        dy = int(abs(dy))

        over=0
    
        if dx > dy:
            for _ in range(dx):
                self.xstepper.move(dirx, 1)
                over+=dy
                if over >= dx:
                    over -= dx
                    self.ystepper.move(diry, 1)
        else:
            for _ in range(dy):
                self.ystepper.move(diry, 1)
                over+=dx
                if over >= dy:
                    over -= dy
                    self.xstepper.move(dirx, 1)

    def exit(self):
        self.xstepper.unhold()
        self.ystepper.unhold()

class GCodeParser:
    def __init__(self, control):
        self.control = control
        self.xs = control.xstepper
        self.ys = control.ystepper
        self.mode_abs=True

    def process(self, input):
        for line in input:
            gcode = self.parsenumber(line, "G")
            if gcode in (01, 02): # move in a line
                self.control.line(self.parsenumber(line, "X", self.xs.pos if self.mode_abs else 0),
                                  self.parsenumber(line, "Y", self.ys.pos if self.mode_abs else 0))
                continue
            elif gcode == 92: # set logical position
                self.xs.pos = self.parsenumber("X", 0)
                self.ys.pos = self.parsenumber("Y", 0)
                continue
            elif gcode == 90:
                self.mode_abs = True
                continue
            elif gcode == 91:
                self.mode_abs = False
                continue
            elif gcode == 4:
                timer.sleep(self.parsenumber(line, "P", 0) * 1000)
                continue

            mcode = self.parsenumber(line, "M")
            if mcode == 18:
                self.control.exit()
                continue

    def parsenumber(self, line, code, default=None):
        idx = line.find(code)
        if idx < 0: return default

        pat = re.compile("[+-]?\d+(.\d+)?")
        match = pat.search(line, idx+1)
        if match == None: return default
        return float(match.group(0))
    
input = io.open(sys.argv[1])
control = Control()
parser = GCodeParser(control)
parser.process(input)
input.close()
control.exit()
