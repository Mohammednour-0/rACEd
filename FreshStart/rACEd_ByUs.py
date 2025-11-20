#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

motorDrive = Motor(Port.B)
motorRoll = Motor(Port.C)
motorBitch = Motor(Port.A)


sensorTouch = TouchSensor(Port.S1)
sensorColor = ColorSensor(Port.S2)
sensorUltrasonic = UltrasonicSensor(Port.S4)


# Write your program here.
ev3.speaker.beep()

while True:
    distance = float(sensorUltrasonic.distance())
    reflection = float(sensorColor.reflection())

    if distance > 76:
        angleUltrasonic = (distance - 76) / 1.55    #Front
    else:
        angleUltrasonic = (distance - 76) / 1.35    #Back

    if reflection < 33:
        angleColor = (reflection - 33) / 0.55   #Left
    else:
        angleColor = (reflection - 33) / 0.8    #Right

    
    positionBitch = int(round(float(angleUltrasonic) * -4.16))

    if positionBitch > 75:
        positionBitch = 75
    elif positionBitch < -75:
        positionBitch = -75

    motorBitch.run_target(50, positionBitch, then=Stop.HOLD, wait=True)

    positionRoll = int(round(float(angleColor) * 4))

    if positionRoll > 80:
        positionRoll = 80
    elif positionRoll < -80:
        positionRoll = -80

    motorRoll.run_target(50, positionRoll, then=Stop.HOLD, wait=True)


    print("Ultra:", angleUltrasonic)
    print("Reflection:", angleColor)