from machine import Pin, PWM
import sys
import struct


# Define PWM and EN pin mappings
motor_pwm_pins = [20, 14, 6, 0, 4]   # M1–M5 PWM
motor_en_pins  = [21, 15, 7, 1, 5]   # M1–M5 EN
motor_ns_pins  = [19]   # NSLEEP


# Initial EN Pin
for pin in motor_en_pins:
   en = Pin(pin, Pin.OUT)
   en.value(1)


# Initial NSLEEP Pin
for pin in motor_ns_pins:
   ns = Pin(pin, Pin.OUT)
   ns.value(1) # No sleep


# Initialize EN pins (set to HIGH to enable motor drivers)
for pin in motor_en_pins:
   en = Pin(pin, Pin.OUT)
   en.value(1)  # Enable motor driver


# Initialize PWM objects
pwms = []
for pin in motor_pwm_pins:
   pwm = PWM(Pin(pin), freq=200)   # 200Hz suitable for ERM vibration motors
   pwm.duty(0)                     # Start with 0% duty
   pwms.append(pwm)


# Main loop: receive 5 float32 values (20 bytes) and update PWM
while True:
#    print("🚀 Ready to receive PWM values from host...")
   try:
       data = sys.stdin.buffer.read(20)
   except:
       continue   


   if not data or len(data) < 20:
       continue


   try:
       values = struct.unpack('<5f', data)
   except:
       continue


   for i in range(5):
       # Clamp input to [0.0, 1.0], then map to duty cycle 0–1023
       val = max(0.0, min(1.0, values[i]))
       duty = int(val * 1023)
       pwms[i].duty(duty)