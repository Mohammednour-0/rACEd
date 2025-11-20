#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch

# --- PID Konfiguration ---
# Geschwindigkeit (äußere Schleife)
PID_SPEED_KP = 4.0
PID_SPEED_KI = 1.02
PID_SPEED_KD = 5.0

# Position (innere Schleife) – regelt das Wackeln weg
PID_POS_KP = 1.8
PID_POS_KI = 0.2
PID_POS_KD = 0.6

# Begrenzungen
MIN_SPEED = 5
MAX_SPEED = 50

ev3 = EV3Brick()

# Motoren
motorRoll = Motor(Port.C)
motorPitch = Motor(Port.A)

# Sensoren
sensorTouch = TouchSensor(Port.S1)
sensorColor = ColorSensor(Port.S2)
sensorUltrasonic = UltrasonicSensor(Port.S4)

# --- PID-Klasse (Allgemein verwendbar) ---
class PID:
    def __init__(self, kp, ki, kd, min_out=None, max_out=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.last_error = 0

    def compute(self, error, dt):
        # Integral mit Anti-Windup
        if abs(error) < 0.5:
            self.integral = 0
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.last_error = error

        # Begrenzen (Clamping)
        if self.min_out is not None and output < self.min_out:
            output = self.min_out
        if self.max_out is not None and output > self.max_out:
            output = self.max_out

        return output

# Zwei verschachtelte PID-Regler (pro Achse)
pid_speed_pitch = PID(PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, MIN_SPEED, MAX_SPEED)
pid_speed_roll  = PID(PID_SPEED_KP, PID_SPEED_KI, PID_SPEED_KD, MIN_SPEED, MAX_SPEED)
pid_position_pitch = PID(PID_POS_KP, PID_POS_KI, PID_POS_KD)
pid_position_roll  = PID(PID_POS_KP, PID_POS_KI, PID_POS_KD)

# Timer starten
watch = StopWatch()
watch.reset()
ev3.speaker.beep()

print("Doppelte PID-Stabilisierung aktiv...")

while True:
    dt = watch.time() / 1000.0
    watch.reset()

    # --- Sensorwerte ---
    distance = float(sensorUltrasonic.distance())
    reflection = float(sensorColor.reflection())

    # --- Winkelberechnung ---
    if distance > 76:
        angleUltrasonic = (distance - 76) / 1.55
    else:
        angleUltrasonic = (distance - 76) / 1.35

    if reflection < 33:
        angleColor = (reflection - 32) / 0.55
    else:
        angleColor = (reflection - 32) / 0.65

    # --- Ziel-Positionen ---
    positionPitch = int(round(angleUltrasonic * -4.16))
    positionRoll = int(round(angleColor * 4))

    # Begrenzen
    positionPitch = max(min(positionPitch, 75), -75)
    positionRoll  = max(min(positionRoll, 80), -80)

    # --- PID 1: Position ---
    # Fehler = Sollwinkel - Istwinkel
    current_pitch = motorPitch.angle()
    current_roll = motorRoll.angle()
    error_pitch = positionPitch - current_pitch
    error_roll = positionRoll - current_roll

    # Positions-PID gibt gewünschte Geschwindigkeit vor (Zielgeschwindigkeit)
    target_speed_pitch = pid_position_pitch.compute(error_pitch, dt)
    target_speed_roll = pid_position_roll.compute(error_roll, dt)

    # --- PID 2: Geschwindigkeit ---
    # Dieser PID begrenzt und glättet die Geschwindigkeit
    final_speed_pitch = pid_speed_pitch.compute(target_speed_pitch, dt)
    final_speed_roll = pid_speed_roll.compute(target_speed_roll, dt)

    # --- Motoren bewegen ---
    motorPitch.run(final_speed_pitch)
    motorRoll.run(final_speed_roll)

    wait(10)
