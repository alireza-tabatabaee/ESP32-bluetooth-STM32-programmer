# Companion Python code with the ESP32 bluetooth STM32 programmer, A. Tabatabaee, 2022
# usage: python senddata.py -d file.hex -p COM18
# pass the hex file with the -d handle and the virtual bluetooth COM port with the -p handle
import serial
import time
import sys
from intelhex import IntelHex
import argparse
parser = argparse.ArgumentParser()
parser.add_argument('-p', '--port', help='Bluetooth virtual port')
parser.add_argument('-d', '--data', help='.hex file to read from')
args = parser.parse_args()

if(args.port == None):
    raise Exception("Please provide the virtual bluetooth COM port with the -p flag")
else:
    COMPORT = args.port
if(args.data == None):
    raise Exception("Please provide the .hex file with the -d flag")
else:
    FILENAME = args.data

ser = serial.Serial(COMPORT);
ser.write(b'INIT\0')
print("Erasing all flash sectors...")
ih = IntelHex(FILENAME)

while(True):
    x = ser.read(4)
    x = x.decode()
    if(x == 'ACK0'):
        print("Flash erase successful");
        contents = ih.tobinarray(start=0x08000000)
        for i in range(0,int(len(contents)/256)+1):
            endv = min((i+1)*256, len(contents))
            ser.write(contents[i*256 : endv])
            x = ser.read(4).decode()
            if(x == 'ACK1'):
                print(f"{i+1}/{int(len(contents)/256)+1} done");
            else:
                raise Exception(x)
        
        ser.write(b'FINI\0')
        x = ser.read(4).decode()
        if(x == 'ACK2'):
            print("Successfully Flashed. Reset the board with BOOT0=0 to run");
            break
            
