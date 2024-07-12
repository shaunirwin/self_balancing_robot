import serial
import struct
import numpy as np
from enum import Enum

ser = serial.Serial(port='/dev/ttyACM0',
    baudrate=115200,
    timeout=5)      # timeout in [sec]

ser.reset_input_buffer()

msg_type = 'state_estimate'

    # float pitch_setpoint;
    # float pitch_current;
    # float pitch_error;

    # float motorSpeed;
    # int motorDir;
    # uint dutyCycle;

msg_formats = {
    'raw_imu_data': '<hhhhhh',  # imu raw data: accel {x,y,z}, gyro {x,y,z}
    'state_estimate': '<fffllll',
    'control_packet': '<ff' #'<ffff' #'<ffffiI'
}

msg_format = msg_formats[msg_type]
print('msg_format:', msg_format)

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
wheel_velocity_measurement_timesteps = 25   # num estimator time steps that encoder pulses are recorded for velocity

gyro_raw_hist = []      # store initial guro values to calculate offset
gyro_offset = [0, 0, 0]     # deg/sec
pitch_angle_gyro = 0    # rad
gyro_is_calibrated = False

# complementary filter
alpha = 0.98
pitch_angle_est = 0

wheel_diameter = 0.0815     # [m]
pulses_per_revolution = 464
distance_per_pulse = wheel_diameter * np.pi / pulses_per_revolution   # [m]

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
            if msg_type == 'state_estimate':
                pitch_angle_accel = data_unpacked[0]
                pitch_angle_gyro = data_unpacked[1]
                pitch_angle_est = data_unpacked[2]
                motor_1_encoder_count = data_unpacked[3]
                motor_1_encoder_count_delta = data_unpacked[4]
                motor_2_encoder_count = data_unpacked[5]
                motor_2_encoder_count_delta = data_unpacked[6]
                # motor_1_dir_meas = data_unpacked[4]
                # motor_2_encoder_count = data_unpacked[5]
                # motor_2_dir_meas = data_unpacked[6]

                wheel_1_vel = motor_1_encoder_count_delta * distance_per_pulse * imu_sample_freq / wheel_velocity_measurement_timesteps     # [m/s]
                wheel_2_vel = motor_2_encoder_count_delta * distance_per_pulse * imu_sample_freq / wheel_velocity_measurement_timesteps     # [m/s]
                        
                #print(accel_xyz, gyro_xyz, pitch_angle_accel*180/np.pi) #, msg)#, ser_bytes.decode("utf-8"))
                print(f'pitch angle accel [deg]: {pitch_angle_accel*180/np.pi:+3.2f}, '
                    f'pitch angle gyro [deg]: {pitch_angle_gyro*180/np.pi:+3.2f}, '
                    f'pitch angle est [deg]: {pitch_angle_est*180/np.pi:+3.2f}, '
                    f'wheel1 dist [m]: {motor_1_encoder_count * distance_per_pulse:+3.3f}, '
                    f'wheel2 dist [m]: {motor_2_encoder_count * distance_per_pulse:+3.3f}, '
                    f'wheel1 vel [m/s]: {wheel_1_vel:+1.3f}, '
                    f'wheel2 vel [m/s]: {wheel_2_vel:+1.3f}')
                    #   f'motor1: {motor_1_encoder_count} ({motor_1_dir_meas}), '
                    #   f'motor2: {motor_2_encoder_count} ({motor_2_dir_meas})')
            elif msg_type == 'control_packet':
                pitch_setpoint = data_unpacked[0]
                pitch_current = data_unpacked[1]
                # pitch_error = data_unpacked[2]
                # motorSpeed = data_unpacked[3]
                # motorDir = data_unpacked[4]
                # dutyCycle = data_unpacked[5]

                pitch_error = -1.
                motorSpeed = -1.
                motorDir = -1.
                dutyCycle = -1.

                print(f'pitch_setpoint [deg]: {pitch_setpoint*180/np.pi:+3.2f}, '
                    f'pitch_current [deg]: {pitch_current*180/np.pi:+3.2f}, '
                    f'pitch_error [deg]: {pitch_error*180/np.pi:+3.2f}, '
                    f'motorSpeed []: {motorSpeed:+3.2f}, '
                    f'motorDir: {motorDir:+1d}, '
                    f'dutyCycle: {dutyCycle:+3.2f}')
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
