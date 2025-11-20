#!/usr/bin/env python3
# HYBRID PID + Q-LEARNING BALANCE ROBOT (ev3dev2.3.5)
# -------------------------------------------------

from ev3dev2.sensor.lego import ColorSensor, UltrasonicSensor
from ev3dev2.motor import Motor, SpeedPercent, OUTPUT_A, OUTPUT_B, OUTPUT_C
from ev3dev2.sound import Sound
from time import sleep
import json
import os
import random

sound = Sound()

# Sensoren
cs = ColorSensor('in2')          # Pendel Pitch (Helligkeit)
us = UltrasonicSensor('in4')     # Pendel Roll (Distanz)

# Motoren
drive = Motor(OUTPUT_B)          # Vorwärts
pitch = Motor(OUTPUT_A)          # Balance vorne/hinten
roll = Motor(OUTPUT_C)           # Balance links/rechts

# ------------------------------------------------------------
# 1. PID CONTROLLER
# ------------------------------------------------------------

# PID Startparameter (werden später von KI verbessert)
Kp_pitch = 0.05     # extrem reduziert
Ki_pitch = 0.001
Kd_pitch = 0.02

Kp_roll = 0.05      # extrem reduziert
Ki_roll = 0.001
Kd_roll = 0.02

# PID Fehlerwerte
last_pitch_error = 0
pitch_integral = 0

last_roll_error = 0
roll_integral = 0


def pid_control(error, last_error, integral, Kp, Ki, Kd):
    integral += error
    derivative = error - last_error
    output = Kp * error + Ki * integral + Kd * derivative
    return output, error, integral


# ------------------------------------------------------------
# 1b. ADAPTIVE SENSOR CALIBRATION (AI-gesteuert)
# ------------------------------------------------------------

CALIB_FILE = "sensor_calib.json"

# Adaptive Kalibrierungsparameter laden
try:
    with open(CALIB_FILE, "r") as f:
        calib_data = json.load(f)
        pitch_neutral = calib_data.get("pitch_neutral", 33.0)
        pitch_scale = calib_data.get("pitch_scale", 1.4)
        roll_neutral = calib_data.get("roll_neutral", 6.7)
        roll_scale = calib_data.get("roll_scale", 7.7)
except:
    # Startparameter (werden trainiert)
    pitch_neutral = 33.0
    pitch_scale = 1.4
    roll_neutral = 6.7
    roll_scale = 7.7

# Historie für adaptives Lernen
pitch_history = []
roll_history = []
max_history = 100

# Lernraten für adaptive Kalibrierung (sicher kleine Schritte)
calib_lr_neutral_pitch = 0.001
calib_lr_scale_pitch = 0.0005
calib_lr_neutral_roll = 0.0005
calib_lr_scale_roll = 0.0002

# Mindest-/Höchstwerte
PITCH_SCALE_MIN, PITCH_SCALE_MAX = 0.5, 3.0
ROLL_SCALE_MIN, ROLL_SCALE_MAX = 0.5, 15.0


def save_calib():
    global pitch_neutral, pitch_scale, roll_neutral, roll_scale
    with open(CALIB_FILE, "w") as f:
        json.dump({
            "pitch_neutral": pitch_neutral,
            "pitch_scale": pitch_scale,
            "roll_neutral": roll_neutral,
            "roll_scale": roll_scale
        }, f)


def adaptive_calibration():
    """Online gradient-descent style Anpassung der Kalibrierungsparameter.

    Verwendet die in `pitch_history` / `roll_history` gesammelten Rohwerte
    und die daraus berechneten Fehler, berechnet Gradienten und passt
    `neutral` und `scale` in kleinen Schritten an. Sicherheitsgrenzen
    verhindern extreme Anpassungen.
    """
    global pitch_neutral, pitch_scale, roll_neutral, roll_scale

    n_p = len(pitch_history)
    n_r = len(roll_history)
    if n_p < 8 and n_r < 8:
        return  # zu wenig Daten

    # --- Pitch: gradientenbasierte Updates ---
    if n_p >= 8:
        # pitch_history enthält Tupel (raw, error)
        raws = [h[0] for h in pitch_history]
        errors = [h[1] for h in pitch_history]

        # mittlere Gradienten (über alle Samples)
        sum_e = sum(errors)
        sum_e_x = sum(e * (raw - pitch_neutral) for (raw, e) in pitch_history)

        # Update neutral: d/d(neutral) of e^2 = 2*e*(-scale)
        delta_neutral = calib_lr_neutral_pitch * (sum_e * pitch_scale) / max(1, n_p)
        pitch_neutral += delta_neutral

        # Update scale: d/d(scale) of e^2 = 2*e*(raw-neutral)
        delta_scale = -calib_lr_scale_pitch * (sum_e_x) / max(1, n_p)
        pitch_scale += delta_scale

        # Clamp scale
        old_pitch_scale = pitch_scale
        pitch_scale = max(PITCH_SCALE_MIN, min(PITCH_SCALE_MAX, pitch_scale))

        # Debug
        try:
            print("[CALIB] Pitch delta_neutral={:+.4f}, delta_scale={:+.4f} -> neutral={:.3f}, scale={:.3f}".format(
                delta_neutral, delta_scale, pitch_neutral, pitch_scale
            ))
        except Exception:
            pass

    # --- Roll: gradientenbasierte Updates ---
    if n_r >= 8:
        raws_r = [h[0] for h in roll_history]
        errors_r = [h[1] for h in roll_history]

        sum_e_r = sum(errors_r)
        sum_e_x_r = sum(e * (raw - roll_neutral) for (raw, e) in roll_history)

        delta_neutral_r = calib_lr_neutral_roll * (sum_e_r * roll_scale) / max(1, n_r)
        roll_neutral += delta_neutral_r

        delta_scale_r = -calib_lr_scale_roll * (sum_e_x_r) / max(1, n_r)
        roll_scale += delta_scale_r

        old_roll_scale = roll_scale
        roll_scale = max(ROLL_SCALE_MIN, min(ROLL_SCALE_MAX, roll_scale))

        try:
            print("[CALIB] Roll delta_neutral={:+.4f}, delta_scale={:+.4f} -> neutral={:.3f}, scale={:.3f}".format(
                delta_neutral_r, delta_scale_r, roll_neutral, roll_scale
            ))
        except Exception:
            pass

    # Historie zurücksetzen und speichern
    pitch_history.clear()
    roll_history.clear()
    save_calib()


# ------------------------------------------------------------
# 2. Q-LEARNING SETUP (KI)
# ------------------------------------------------------------

QFILE = "qtablePID.json"

# Q laden
try:
    with open(QFILE, "r") as f:
        Q = json.load(f)
except:
    Q = {}

ACTIONS = [-2, -1, 0, 1, 2]  # KI-Anpassung der PID-Werte


def save_q():
    with open(QFILE, "w") as f:
        json.dump(Q, f)


def get_state(pitch_err, roll_err):
    # Sehr feine Auflösung: 0.2° / 0.2cm
    return (round(pitch_err / 0.2), round(roll_err / 0.2))


def choose_action(state):
    s = str(state)

    if s not in Q:
        Q[s] = {str(a): 0 for a in ACTIONS}

    # 20% zufällig (exploration)
    if random.random() < 0.2:
        return random.choice(ACTIONS)

    # 80% beste Aktion
    best = max(Q[s], key=Q[s].get)
    return int(best)


def update_q(state, action, reward):
    s = str(state)
    a = str(action)

    if s not in Q:
        Q[s] = {str(x): 0 for x in ACTIONS}

    Q[s][a] += 0.1 * (reward - Q[s][a])


# ------------------------------------------------------------
# 3. SENSOR → NEIGUNG (mit adaptiver Kalibrierung)
# ------------------------------------------------------------

def get_pitch():
    """Lichtsensor-Lesung mit adaptiver Kalibrierung"""
    global pitch_history
    intensity = cs.reflected_light_intensity
    pitch_angle = (intensity - pitch_neutral) * pitch_scale
    
    # Fehler für adaptives Lernen sammeln (Annahme: kleine Fehler bei Balance)
    pitch_history.append((intensity, pitch_angle))
    if len(pitch_history) > max_history:
        pitch_history.pop(0)
    
    return pitch_angle


def get_roll():
    """Ultraschall-Lesung mit adaptiver Kalibrierung"""
    global roll_history
    dist = us.distance_centimeters
    if dist is None:
        return 0
    
    roll_angle = (dist - roll_neutral) * roll_scale
    
    # Fehler für adaptives Lernen sammeln
    roll_history.append((dist, roll_angle))
    if len(roll_history) > max_history:
        roll_history.pop(0)
    
    return roll_angle


# ------------------------------------------------------------
# 4. MAIN LOOP
# ------------------------------------------------------------

def main():
    global Kp_pitch, Kp_roll
    global last_pitch_error, last_roll_error
    global pitch_integral, roll_integral
    global pitch_neutral, pitch_scale, roll_neutral, roll_scale

    sound.speak("PID plus learning active")

    drive.on(SpeedPercent(30))  # konstante Vorwärtsfahrt
    
    loop_counter = 0
    calib_interval = 100  # Alle 100 Zyklen Kalibrierung trainieren (kontinuierlicheres Lernen)

    while True:
        # ----- Sensors -----
        pitch_err = get_pitch()
        roll_err = get_roll()

        # ----- PID Steuerung -----
        p_out, last_pitch_error, pitch_integral = pid_control(
            pitch_err, last_pitch_error, pitch_integral,
            Kp_pitch, Ki_pitch, Kd_pitch
        )

        r_out, last_roll_error, roll_integral = pid_control(
            roll_err, last_roll_error, roll_integral,
            Kp_roll, Ki_roll, Kd_roll
        )

        # Motorbewegung (ev3dev2: on mit SpeedPercent)
        # Output auf 10-70% begrenzen, in 2%-Schritten
        p_speed = int(p_out * 0.05)  # noch langsamer (20x langsamer als original)
        r_speed = int(r_out * 0.05)
        
        # Begrenzen auf 10-70% und runden auf 2%-Schritte
        def clamp_speed(s):
            s = max(10, min(70, abs(s)))  # 10-70%
            s = round(s / 2) * 2  # auf 2%-Schritte runden
            return s
        
        p_speed = clamp_speed(p_speed) if p_out >= 0 else -clamp_speed(-p_speed)
        r_speed = clamp_speed(r_speed) if r_out >= 0 else -clamp_speed(-r_speed)
        
        pitch.on(SpeedPercent(p_speed))
        roll.on(SpeedPercent(r_speed))

        # ------ KI Entscheidung (PID-Anpassung) ------
        state = get_state(pitch_err, roll_err)
        action = choose_action(state)

        # KI modifiziert Kp leicht
        Kp_pitch += action * 0.05
        Kp_roll  += action * 0.05

        # Begrenzen
        Kp_pitch = max(0.5, min(6, Kp_pitch))
        Kp_roll  = max(0.5, min(6, Kp_roll))

        # -------- Belohnung --------
        error_total = abs(pitch_err) + abs(roll_err)
        reward = -error_total

        update_q(state, action, reward)
        save_q()
        
        # ------ ADAPTIVE SENSORKALIBRIERUNG (alle 500ms) ------
        loop_counter += 1
        if loop_counter >= calib_interval:
            adaptive_calibration()
            loop_counter = 0

        sleep(0.02)  # 20ms delay


try:
    main()

except KeyboardInterrupt:
    drive.stop()
    save_q()
    save_calib()
    sound.speak("Learning saved")
