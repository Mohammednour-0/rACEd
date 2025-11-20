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

# PID Klasse zur Berechnung der Geschwindigkeit
class SpeedPID:
    def __init__(self, kp, ki, kd, min_out, max_out):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out = min_out
        self.max_out = max_out
        self.integral = 0
        self.last_error = 0

    def compute_speed(self, error, dt):
        # Absoluten Fehler nutzen, da Geschwindigkeit immer positiv ist
        abs_error = abs(error)
        
        # Reset Integral wenn Fehler sehr klein ist (gegen Aufschaukeln)
        if abs_error < 1:
            self.integral = 0
        
        self.integral += abs_error * dt
        derivative = (abs_error - self.last_error) / dt if dt > 0 else 0
        
        # PID Berechnung
        speed = (abs_error * self.kp) + (self.integral * self.ki) + (derivative * self.kd)
        
        self.last_error = abs_error
        
        # Begrenzen (Clamping)
        if speed < self.min_out: speed = self.min_out
        if speed > self.max_out: speed = self.max_out
        
        return int(speed)

# Zwei PID Regler erstellen (für Pitch und Roll)
pid_speed_pitch = SpeedPID(PID_KP, PID_KI, PID_KD, MIN_SPEED, MAX_SPEED)
pid_speed_roll  = SpeedPID(PID_KP, PID_KI, PID_KD, MIN_SPEED, MAX_SPEED)

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

    # 4. PID berechnet die nötige GESCHWINDIGKEIT basierend auf dem Fehler (Angle)
    # Je größer der Winkel-Fehler, desto schneller fahren wir zum Ziel.
    speed_pitch = pid_speed_pitch.compute_speed(angleUltrasonic, dt)
    speed_roll  = pid_speed_roll.compute_speed(angleColor, dt)

    # 5. Motoren bewegen
    # wait=False ist entscheidend für harmonische Bewegung!
    # Beide Motoren erhalten ihr Ziel und bewegen sich gleichzeitig.
    # Stop.HOLD sorgt dafür, dass sie die Position halten, wenn sie ankommen.
    motorPitch.run_target(speed_pitch, positionPitch, then=Stop.HOLD, wait=False)
    motorRoll.run_target(speed_roll, positionRoll, then=Stop.HOLD, wait=False)

    # Kleines Delay für Stabilität
    wait(10)