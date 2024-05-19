import serial
import struct
import numpy as np
from enum import Enum

ser = serial.Serial(port='/dev/ttyACM0',
    baudrate=115200,
    timeout=5)      # timeout in [sec]

ser.reset_input_buffer()

msg_format = '<hhhhhh'
msg_size = struct.calcsize(msg_format)        # accel {x,y,z}, gyro {x,y,z}
print('struct is this size [bytes]:', msg_size)

# serial control characters
STX = b'!' #0x02   # start of frame
ETX = b'@' #'\x03'   # end of frame

accel_range = 2     # +-2g
gyro_range = 250    # +- 250 deg/sec
accel_resolution = accel_range / 16384.0
gyro_resolution = gyro_range / 16384.0
imu_sample_freq = 250   # Hz

gyro_raw_hist = []      # store initial guro values to calculate offset
gyro_offset = [0, 0, 0]

#class GyroState(Enum):
#    STORING_CALIB_VALUES = 1
#    CALCULATE_OFFSET = 2
#    CALIBRATED = 3

#gyro_state = GyroState.STORING_CALIB_VALUES
gyro_is_calibrated = False

while True:
    try:
        ser_bytes = ser.read(1)
        
        #print(ser_bytes, STX, ser_bytes == STX, 'STX')
        
        if ser_bytes != STX:
            continue
            
        # read the frame
        msg = ser.read(msg_size)
        # read the end of frame
        ser_bytes = ser.read(1)
        
        #print(ser_bytes)
        
        if ser_bytes != ETX:
            continue
            
        #print('valid packet received!')

        data_unpacked = struct.unpack(msg_format, msg)
        accel_raw = data_unpacked[:3]
        gyro_raw = data_unpacked[3:]
        accel_xyz = [v * accel_resolution/2 for v in accel_raw]     # m/s^2
        gyro_xyz = [v * gyro_resolution/2 for v in gyro_raw]        # deg/sec
        pitch_angle_accel = np.arctan2(accel_xyz[0], accel_xyz[2])  # rad
        pitch_angle_gyro = 0    # rad
        
        if not gyro_is_calibrated: #gyro_state == GyroState.STORING_CALIB_VALUES:
            if len(gyro_raw_hist) < imu_sample_freq:
                gyro_raw_hist.append(gyro_xyz)
                
            if len(gyro_raw_hist) == imu_sample_freq:
                gyro_offset = list(np.mean(gyro_raw_hist, axis=0))
                print('calculated gyro offset:', gyro_offset)
                gyro_is_calibrated = True
                #exit(0)
        print(accel_xyz, gyro_xyz, pitch_angle_accel*180/np.pi) #, msg)#, ser_bytes.decode("utf-8"))
        #print(f'pitch angle [deg]: {pitch_angle_accel*180/np.pi:.2f}')
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
