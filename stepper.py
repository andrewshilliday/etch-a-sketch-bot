import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

class Stepper():
    phases = [(1,0,1,0), (0,1,1,0), (0,1,0,1), (1,0,0,1)]
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
            next_idx=(self.idx + dir)%len(self.phases)

            for (pin,state) in zip(self.pins, phases[next_idx]):
                GPIO.output(pin, state)

            self.idx = next_idx
            self.dir = dir
            self.pos += dir

            time.sleep(delay)

    def unhold(self):
        for pin in self.pins:
            GPIO.output(pin, 0)
