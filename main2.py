#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

# --- PID Konfiguration für die GESCHWINDIGKEIT ---
# KP: Wie aggressiv wird beschleunigt bei Fehler?
# KI: Hilft bei konstanten kleinen Fehlern (hier eher niedrig halten)
# KD: Bremst ab, wenn sich der Fehler schnell ändert
PID_KP = 4.0
PID_KI = 1.02
PID_KD = 5.0

# Minimale und maximale Motor-Geschwindigkeit
MIN_SPEED = 5   # Damit er sich bei kleinen Fehlern überhaupt bewegt
MAX_SPEED = 50  # Nicht zu schnell, um Überschwingen zu vermeiden

ev3 = EV3Brick()

# Motoren
motorRoll = Motor(Port.D)
motorPitch = Motor(Port.A) # (Ehemals motorBitch)

# Sensoren
sensorTouch = TouchSensor(Port.S1)
sensorColor = ColorSensor(Port.S2)
sensorUltrasonic = UltrasonicSensor(Port.S4)

# PID-Regler für Positionsregelung
class PositionPID:
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error
        # Begrenzen
        if output < self.min_out:
            output = self.min_out
        if output > self.max_out:
            output = self.max_out
        return int(output)

# Zwei PID Regler für Positionsregelung (Pitch und Roll)
pid_pitch = PositionPID(PID_KP, PID_KI, PID_KD, -MAX_SPEED, MAX_SPEED)
pid_roll  = PositionPID(PID_KP, PID_KI, PID_KD, -MAX_SPEED, MAX_SPEED)

watch = StopWatch()
watch.reset()
ev3.speaker.beep()

print("Stabilisierung gestartet...")

while True:
    # Zeitdifferenz für PID messen
    dt = watch.time() / 1000.0
    watch.reset()

    # 1. Sensordaten lesen
    distance = float(sensorUltrasonic.distance())
    reflection = float(sensorColor.reflection())

    # 2. Deine Logik zur Winkelberechnung (Fehlerberechnung)
    if distance > 76:
        angleUltrasonic = (distance - 76) / 1.55
    else:
        angleUltrasonic = (distance - 76) / 1.35

    if reflection < 33:
        angleColor = (reflection - 32) / 0.55
    else:
        angleColor = (reflection - 32) / 0.65

    # 3. Ziel-Positionen berechnen (Deine Formeln)
    positionPitch = int(round(float(angleUltrasonic) * -4.16))
    positionRoll = int(round(float(angleColor) * 4))

    # Begrenzungen
    if positionPitch > 75: positionPitch = 75
    elif positionPitch < -75: positionPitch = -75

    if positionRoll > 80: positionRoll = 80
    elif positionRoll < -80: positionRoll = -80

    # 4. PID berechnet die Geschwindigkeit aus Positionsfehler
    error_pitch = positionPitch - motorPitch.angle()
    error_roll = positionRoll - motorRoll.angle()
    speed_pitch = pid_pitch.compute(error_pitch, dt)
    speed_roll = pid_roll.compute(error_roll, dt)

    # 5. Motoren direkt mit berechneter Geschwindigkeit ansteuern
    motorPitch.run(speed_pitch)
    motorRoll.run(speed_roll)

    # Kleines Delay für Stabilität
    wait(10)