import serial, time

ser = serial.Serial("/dev/ttyACM0", 115200)  
time.sleep(1)                          

def moteurs(vg, vd):
    ser.write(f"{int(vg)},{int(vd)}\n".encode())

moteurs(70, 70)    # avance
ser.close()