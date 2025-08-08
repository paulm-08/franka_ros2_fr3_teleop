from machine import Pin, PWM
import sys
import struct
import time

motor_pwm_pins = [20, 14, 6, 0, 4]
motor_en_pins  = [21, 15, 7, 1, 5]
motor_ns_pins  = [19]

# Enable drivers
for pin in motor_en_pins:
    Pin(pin, Pin.OUT).value(1)
for pin in motor_ns_pins:
    Pin(pin, Pin.OUT).value(1)

# PWM setup
pwms = [PWM(Pin(pin), freq=200) for pin in motor_pwm_pins]
for pwm in pwms:
    pwm.duty(0)

# Serial receive buffer
buf = b""

while True:
    # Read any available bytes from stdin
    try:
        chunk = sys.stdin.buffer.read(20)  # read up to 20 bytes
        if chunk:
            buf += chunk
            last_update = time.ticks_ms()
    except:
        print("Buffer read failed")
        continue

    # Process all complete packets
    while len(buf) >= 20:
        packet = buf[:20]
        buf = buf[20:]  # remove processed packet

        try:
            values = struct.unpack('<5f', packet)
        except:
            continue

        for i in range(5):
            val = max(0.0, min(1.0, values[i]))
            duty = int(val * 1023)
            pwms[i].duty(duty)

    # # failsafe
    # if time.ticks_diff(time.ticks_ms(), last_update) > 200:
    #     for p in pwms:
    #         p.duty(0)
    #         break
