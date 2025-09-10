import serial
import struct
import time
import numpy as np

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# time.sleep(2)  # Let ESP32 boot
# ser.reset_input_buffer()  # discard any leftover bytes

try:
    for m in range(5):

        for i in range(20):
            # Example: oscillating intensities
            val=0.25 + 0.25*np.sin(i/2.0)
            # if val<0.05:
            #     val=0.
            pwm_vals = [0.] * 5
            pwm_vals[m] = val
            packet = struct.pack('<5f', *pwm_vals)
            ser.write(packet)
            time.sleep(0.1)  # 30 Hz update rate
            print("✅ Sent:", pwm_vals)
finally:
    time.sleep(2.0)
    pwm_vals = [0.] * 5
    packet = struct.pack('<5f', *pwm_vals)
    ser.write(packet)
    print("✅ Sent:", pwm_vals)

pwm_vals = [0.] * 5
packet = struct.pack('<5f', *pwm_vals)
ser.write(packet)
print("✅ Sent:", pwm_vals)
# ser.reset_input_buffer()  # discard any leftover bytes
# ser.close()
