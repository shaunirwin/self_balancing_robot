import tkinter as tk
import requests

class App:
    def __init__(self, root):
        # get initial status of all variables from robot
        status_info = self.get_request()

        self.root = root
        self.root.title("REST API Client")

        # input fields
        self.label_kp = tk.Label(root, text="Kp")
        self.label_kp.grid(row=0, column=0)
        self.entry_kp = tk.Entry(root)
        self.entry_kp.grid(row=0, column=1)

        self.label_ki = tk.Label(root, text="Ki")
        self.label_ki.grid(row=1, column=0)
        self.entry_ki = tk.Entry(root)
        self.entry_ki.grid(row=1, column=1)

        self.label_kd = tk.Label(root, text="Kd")
        self.label_kd.grid(row=2, column=0)
        self.entry_kd = tk.Entry(root)
        self.entry_kd.grid(row=2, column=1)

        self.label_setpoint = tk.Label(root, text="Setpoint")
        self.label_setpoint.grid(row=3, column=0)
        self.entry_setpoint = tk.Entry(root)
        self.entry_setpoint.grid(row=3, column=1)

        self.label_duty_cycle_min = tk.Label(root, text="MOTOR_DUTY_CYCLE_MIN")
        self.label_duty_cycle_min.grid(row=0, column=3)
        self.entry_duty_cycle_min = tk.Entry(root)
        self.entry_duty_cycle_min.grid(row=0, column=4)

        self.label_duty_cycle_max = tk.Label(root, text="MOTOR_DUTY_CYCLE_MAX")
        self.label_duty_cycle_max.grid(row=1, column=3)
        self.entry_duty_cycle_max = tk.Entry(root)
        self.entry_duty_cycle_max.grid(row=1, column=4)

        self.label_pitch_error_max = tk.Label(root, text="PITCH_ANGLE_ERROR_MAX")
        self.label_pitch_error_max.grid(row=2, column=3)
        self.entry_pitch_error_max = tk.Entry(root)
        self.entry_pitch_error_max.grid(row=2, column=4)

        self.label_pitch_error_min = tk.Label(root, text="PITCH_ANGLE_ERROR_MIN")
        self.label_pitch_error_min.grid(row=3, column=3)
        self.entry_pitch_error_min = tk.Entry(root)
        self.entry_pitch_error_min.grid(row=3, column=4)

        self.label_motor_1_dir_manual = tk.Label(root, text="MOTOR_1_DIR_MANUAL")
        self.label_motor_1_dir_manual.grid(row=0, column=6)
        self.entry_motor_1_dir_manual = tk.Entry(root)
        self.entry_motor_1_dir_manual.grid(row=0, column=7)

        self.label_motor_2_dir_manual = tk.Label(root, text="MOTOR_2_DIR_MANUAL")
        self.label_motor_2_dir_manual.grid(row=1, column=6)
        self.entry_motor_2_dir_manual = tk.Entry(root)
        self.entry_motor_2_dir_manual.grid(row=1, column=7)

        self.label_motor_1_duty_cycle_manual = tk.Label(root, text="MOTOR_1_DUTY_CYCLE_MANUAL")
        self.label_motor_1_duty_cycle_manual.grid(row=2, column=6)
        self.entry_motor_1_duty_cycle_manual = tk.Entry(root)
        self.entry_motor_1_duty_cycle_manual.grid(row=2, column=7)

        self.label_motor_2_duty_cycle_manual = tk.Label(root, text="MOTOR_2_DUTY_CYCLE_MANUAL")
        self.label_motor_2_duty_cycle_manual.grid(row=3, column=6)
        self.entry_motor_2_duty_cycle_manual = tk.Entry(root)
        self.entry_motor_2_duty_cycle_manual.grid(row=3, column=7)

        self.drive_mode = tk.StringVar()
        self.label_drive_mode = tk.Label(root, text="Drive Mode")
        self.label_drive_mode.grid(row=4, column=4)
        self.rb_auto = tk.Radiobutton(root, text='Auto', variable=self.drive_mode, value='AUTO', command=self.on_drive_mode_toggled)
        self.rb_auto.grid(row=4, column=5)
        self.rb_manual = tk.Radiobutton(root, text='Manual', variable=self.drive_mode, value='MANUAL', command=self.on_drive_mode_toggled)
        self.rb_manual.grid(row=4, column=6)

        # buttons
        self.button1 = tk.Button(root, text="Submit Kp", command=self.send_Kp)
        self.button1.grid(row=0, column=2)

        self.button2 = tk.Button(root, text="Submit Ki", command=self.send_Ki)
        self.button2.grid(row=1, column=2)

        self.button3 = tk.Button(root, text="Submit Kd", command=self.send_Kd)
        self.button3.grid(row=2, column=2)

        self.button4 = tk.Button(root, text="Submit setpoint", command=self.send_setpoint)
        self.button4.grid(row=3, column=2)

        self.button5 = tk.Button(root, text="Submit motor duty cycle min", command=self.send_duty_cycle_min)
        self.button5.grid(row=0, column=5)

        self.button6 = tk.Button(root, text="Submit motor duty cycle max", command=self.send_duty_cycle_max)
        self.button6.grid(row=1, column=5)

        self.button7 = tk.Button(root, text="Submit pitch angle err max", command=self.send_pitch_err_max)
        self.button7.grid(row=2, column=5)

        self.button8 = tk.Button(root, text="Submit pitch angle err min", command=self.send_pitch_err_min)
        self.button8.grid(row=3, column=5)


        self.button9 = tk.Button(root, text="Submit motor 1 dir manual", command=self.send_motor1_dir_manual)
        self.button9.grid(row=0, column=8)

        self.button10 = tk.Button(root, text="Submit motor 2 dir manual", command=self.send_motor2_dir_manual)
        self.button10.grid(row=1, column=8)

        self.button11 = tk.Button(root, text="Submit motor 1 duty cycle manual", command=self.send_motor1_duty_cycle_manual)
        self.button11.grid(row=2, column=8)

        self.button12 = tk.Button(root, text="Submit motor 2 duty cycle manual", command=self.send_motor2_duty_cycle_manual)
        self.button12.grid(row=3, column=8)

        # set initial values for each field
        self.entry_kp.insert(0, status_info['PID_Kp'])
        self.entry_ki.insert(0, status_info['PID_Ki'])
        self.entry_kd.insert(0, status_info['PID_Kd'])
        self.entry_setpoint.insert(0, status_info['PID_setpoint'])
        self.entry_duty_cycle_min.insert(0, status_info['MOTOR_DUTY_CYCLE_MIN'])
        self.entry_duty_cycle_max.insert(0, status_info['MOTOR_DUTY_CYCLE_MAX'])
        self.entry_pitch_error_max.insert(0, status_info['PITCH_ANGLE_ERROR_MAX'])
        self.entry_pitch_error_min.insert(0, status_info['PITCH_ANGLE_ERROR_MIN'])
        self.entry_motor_1_dir_manual.insert(0, status_info['MOTOR_1_DIR_MANUAL'])
        self.entry_motor_2_dir_manual.insert(0, status_info['MOTOR_2_DIR_MANUAL'])
        self.entry_motor_1_duty_cycle_manual.insert(0, status_info['MOTOR_1_DUTY_CYCLE_MANUAL'])
        self.entry_motor_2_duty_cycle_manual.insert(0, status_info['MOTOR_2_DUTY_CYCLE_MANUAL'])
        self.drive_mode.set(status_info['CONTROL_MODE'])

    def send_Kp(self):
        self.post_request({'key': 'PID_Kp', 'value': self.entry_kp.get()})

    def send_Ki(self):
        self.post_request({'key': 'PID_Ki', 'value': self.entry_ki.get()})

    def send_Kd(self):
        self.post_request({'key': 'PID_Kd', 'value': self.entry_kd.get()})

    def send_setpoint(self):
        self.post_request({'key': 'PID_setpoint', 'value': self.entry_setpoint.get()})
    
    def send_duty_cycle_min(self):
        self.post_request({'key': 'MOTOR_DUTY_CYCLE_MIN', 'value': self.entry_duty_cycle_min.get()})
    
    def send_duty_cycle_max(self):
        self.post_request({'key': 'MOTOR_DUTY_CYCLE_MAX', 'value': self.entry_duty_cycle_max.get()})
    
    def send_pitch_err_max(self):
        self.post_request({'key': 'PITCH_ANGLE_ERROR_MAX', 'value': self.entry_pitch_error_max.get()})
    
    def send_pitch_err_min(self):
        self.post_request({'key': 'PITCH_ANGLE_ERROR_MIN', 'value': self.entry_pitch_error_min.get()})
    
    def send_motor1_dir_manual(self):
        self.post_request({'key': 'MOTOR_1_DIR_MANUAL', 'value': self.entry_motor_1_dir_manual.get()})
    
    def send_motor2_dir_manual(self):
        self.post_request({'key': 'MOTOR_2_DIR_MANUAL', 'value': self.entry_motor_2_dir_manual.get()})
    
    def send_motor1_duty_cycle_manual(self):
        self.post_request({'key': 'MOTOR_1_DUTY_CYCLE_MANUAL', 'value': self.entry_motor_1_duty_cycle_manual.get()})
    
    def send_motor2_duty_cycle_manual(self):
        self.post_request({'key': 'MOTOR_2_DUTY_CYCLE_MANUAL', 'value': self.entry_motor_2_duty_cycle_manual.get()})
    
    def on_drive_mode_toggled(self):
        self.post_request({'key': 'CONTROL_MODE', 'value': self.drive_mode.get()})

    def post_request(self, data):
        url = 'http://192.168.178.55/set-value'
        
        print('Sending request with body:', data)
        
        try:
            response = requests.post(url, data=data)
            if response.status_code == 200:
                print(f"Request successful: {response.text}")
            else:
                print(f"Request failed with status code: {response.status_code}: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
    
    def get_request(self):
        url = 'http://192.168.178.55/status'
        
        print('Sending get request for url:', url)
        
        try:
            response = requests.get(url)
            if response.status_code == 200:
                print(f"Request successful: {response.text}")
            else:
                print(f"Request failed with status code: {response.status_code}: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
        
        return response.json()

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()

