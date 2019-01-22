import serial
import time

def main():
    try:
        ser = serial.Serial('/dev/ttyUSB0', baudrate = 115200, timeout = 1)  # open serial port
        ser.open()
    except:
        print("\n*** could not open the serial comunication ***\n")


    ser.write(str("PR?\r"))
    time.sleep(0.3)
    bytesToRead = ser.inWaiting()
    print(ser.read(bytesToRead))
    ser.write(str("PR?\r"))
    time.sleep(0.3)
    bytesToRead = ser.inWaiting()
    print(ser.read(bytesToRead))  
    time.sleep(0.5)
    print("===")

    ser.close()
    print(ser.is_open)# close port

if __name__ == "__main__":
    main()