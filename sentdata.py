import serial
import time
import sys
#from intelhex import IntelHex

ser = serial.Serial('COM18');
ser.write(b'RES\0')
ser.write(b'`INIT\0')
chunks = []
ind = -1
#ih = IntelHex()    
#ih.loadhex(sys.argv[1])
#binarr = ih.tobinstr()
while(True):
    if (ser.in_waiting > 0):
        x = ser.read(ser.in_waiting).decode()
        print("X",x)
        if(x=='ACK0'):
            print("ACK0 recieved!");
            binfil = open(sys.argv[1], "rb");
            contents = binfil.read()
            #for i in range(0,int(len(contents)/256)):
            for i in range(0,1):
                chunks.append( contents[i*256: (i+1)*256] )
            #chunks.append( contents[(i+1)*256:] )
            print("sent1", str(len(chunks[0])-1).encode());
            ser.write(str(len(chunks[0])-1).encode())
        
        elif(x=='ACK1'):
            print("ACK1 recieved")
            ind = ind + 1
            print("sent2", chunks[ind]);
            ser.write(chunks[ind])
        elif(x=='ACK2'):
            print("ACK2 recieved")
            if(ind>len(chunks)):
                break

        elif(x.startswith("DATSENT")):
            time.sleep(5)
            print("sent3", str(len(chunks[ind])-1).encode())
            ser.write(str(len(chunks[ind])-1).encode())
        else:
            raise Exception(x)
                
    time.sleep(0.02)        
    