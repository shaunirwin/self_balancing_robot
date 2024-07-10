import serial
import struct
import numpy as np
from enum import Enum

ser = serial.Serial(port='/dev/ttyACM0',
    baudrate=115200,
    timeout=5)      # timeout in [sec]

ser.reset_input_buffer()

receive_raw_imu_data = False

if receive_raw_imu_data:
    msg_format = '<hhhhhh'      # imu raw data: accel {x,y,z}, gyro {x,y,z}
else:
    # msg_format = '<fff'         # state estimate data
    msg_format = '<fffll'         # state estimate data
    # msg_format = '<ffflBlB'         # state estimate data
msg_size = struct.calcsize(msg_format)        
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
gyro_offset = [0, 0, 0]     # deg/sec
pitch_angle_gyro = 0    # rad
gyro_is_calibrated = False

# complementary filter
alpha = 0.98
pitch_angle_est = 0

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
        
        if False:
            data_unpacked = struct.unpack(msg_format, msg)
            accel_raw = data_unpacked[:3]
            gyro_raw = data_unpacked[3:]
            accel_xyz = [v * accel_resolution/2 for v in accel_raw]     # m/s^2
            gyro_xyz = [v * gyro_resolution/2 for v in gyro_raw]        # deg/sec
            
            pitch_angle_accel = np.arctan2(accel_xyz[0], accel_xyz[2])  # rad
            
            if not gyro_is_calibrated:
                if len(gyro_raw_hist) < imu_sample_freq:
                    gyro_raw_hist.append(gyro_xyz)
                    
                if len(gyro_raw_hist) == imu_sample_freq:
                    gyro_offset = list(np.mean(gyro_raw_hist, axis=0))
                    print('calculated gyro offset:', gyro_offset)
                    gyro_is_calibrated = True
                    
            pitch_angular_rate_gyro = (gyro_xyz[1] - gyro_offset[1]) * np.pi / 180.      # rad/sec (calibrated)
            delta_angular_rate_gyro = -pitch_angular_rate_gyro * (1. / imu_sample_freq)        # invert direction to have same sign as pitch_angle_accel
            pitch_angle_gyro += delta_angular_rate_gyro
            
            # complementary filter
            
            pitch_angle_est = alpha * (pitch_angle_est + delta_angular_rate_gyro) + (1-alpha) * pitch_angle_accel
        else:
            data_unpacked = struct.unpack(msg_format, msg)
            pitch_angle_accel = data_unpacked[0]
            pitch_angle_gyro = data_unpacked[1]
            pitch_angle_est = data_unpacked[2]
            motor_1_encoder_count = data_unpacked[3]
            motor_2_encoder_count = data_unpacked[4]
            # motor_1_dir_meas = data_unpacked[4]
            # motor_2_encoder_count = data_unpacked[5]
            # motor_2_dir_meas = data_unpacked[6]
                
        #print(accel_xyz, gyro_xyz, pitch_angle_accel*180/np.pi) #, msg)#, ser_bytes.decode("utf-8"))
        print(f'pitch angle accel [deg]: {pitch_angle_accel*180/np.pi:.2f}, '
              f'pitch angle gyro [deg]: {pitch_angle_gyro*180/np.pi:.2f}, '
              f'pitch angle est [deg]: {pitch_angle_est*180/np.pi:.2f}, '
              f'motor1: {motor_1_encoder_count}, '
              f'motor2: {motor_2_encoder_count}')
            #   f'motor1: {motor_1_encoder_count} ({motor_1_dir_meas}), '
            #   f'motor2: {motor_2_encoder_count} ({motor_2_dir_meas})')
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
