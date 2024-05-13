import serial
import struct

ser = serial.Serial(port='/dev/ttyACM0',
    baudrate=115200,
    timeout=5)      # timeout in [sec]

ser.reset_input_buffer()

size = struct.calcsize('iii')
print('struct is this size [bytes]:', size)

# serial control characters
STX = b'!' #0x02   # start of frame
ETX = b'@' #'\x03'   # end of frame

while True:
    try:
        ser_bytes = ser.read(1)
        
        print(ser_bytes, STX, ser_bytes == STX, 'STX')
        
        if ser_bytes != STX:
            continue
            
        # read the frame
        msg = ser.read(size)
        # read the end of frame
        ser_bytes = ser.read(1)
        
        print(ser_bytes)
        
        if ser_bytes != ETX:
            continue
            
        print('valid packet received!')

        data_unpacked = struct.unpack('<iii', msg)
        print(data_unpacked, msg)#, ser_bytes.decode("utf-8"))
    except:
        print("Keyboard Interrupt")
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.close()
        print('closed connection A')
        
        break
    
ser.reset_input_buffer()
ser.reset_output_buffer()
ser.close()
print('closed connection B')
