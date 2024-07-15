import tkinter as tk
import requests
from math import pi


def deg_to_rad(deg: float):
    return deg * pi / 180.

def rad_to_deg(rad: float):
    return rad * 180. / pi

def writeEntry(entry, text):
    entry.delete(0, tk.END)
    entry.insert(0, text)

class App:
    def __init__(self, root):
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

        self.label_setpoint = tk.Label(root, text="Pitch setpoint [deg]")
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

        self.label_pitch_error_max = tk.Label(root, text="PITCH_ANGLE_ERROR_MAX [deg]")
        self.label_pitch_error_max.grid(row=2, column=3)
        self.entry_pitch_error_max = tk.Entry(root)
        self.entry_pitch_error_max.grid(row=2, column=4)

        self.label_pitch_error_min = tk.Label(root, text="PITCH_ANGLE_ERROR_MIN [deg]")
        self.label_pitch_error_min.grid(row=3, column=3)
        self.entry_pitch_error_min = tk.Entry(root)
        self.entry_pitch_error_min.grid(row=3, column=4)

        self.motor_1_dir_manual = tk.StringVar()
        self.label_motor_1_dir_manual = tk.Label(root, text="MOTOR_1_DIR_MANUAL")
        self.label_motor_1_dir_manual.grid(row=0, column=6)
        self.rb_motor_1_forward = tk.Radiobutton(root, text='Forward', variable=self.motor_1_dir_manual, value='FORWARD', command=self.on_motor_1_dir_toggled)
        self.rb_motor_1_forward.grid(row=0, column=7)
        self.rb_motor_1_reverse = tk.Radiobutton(root, text='Reverse', variable=self.motor_1_dir_manual, value='REVERSE', command=self.on_motor_1_dir_toggled)
        self.rb_motor_1_reverse.grid(row=0, column=8)

        self.motor_2_dir_manual = tk.StringVar()
        self.label_motor_2_dir_manual = tk.Label(root, text="MOTOR_2_DIR_MANUAL")
        self.label_motor_2_dir_manual.grid(row=1, column=6)
        self.rb_motor_2_forward = tk.Radiobutton(root, text='Forward', variable=self.motor_2_dir_manual, value='FORWARD', command=self.on_motor_2_dir_toggled)
        self.rb_motor_2_forward.grid(row=1, column=7)
        self.rb_motor_2_reverse = tk.Radiobutton(root, text='Reverse', variable=self.motor_2_dir_manual, value='REVERSE', command=self.on_motor_2_dir_toggled)
        self.rb_motor_2_reverse.grid(row=1, column=8)

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

        self.label_pitch_current = tk.Label(root, text="Pitch current [deg]")
        self.label_pitch_current.grid(row=4, column=0)
        self.entry_pitch_current = tk.Entry(root)
        self.entry_pitch_current.grid(row=4, column=1)
        self.entry_pitch_current.configure(state='readonly')

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

        self.button11 = tk.Button(root, text="Submit motor 1 duty cycle manual", command=self.send_motor1_duty_cycle_manual)
        self.button11.grid(row=2, column=8)

        self.button12 = tk.Button(root, text="Submit motor 2 duty cycle manual", command=self.send_motor2_duty_cycle_manual)
        self.button12.grid(row=3, column=8)

        self.button13 = tk.Button(root, text="STOP", command=self.send_stop)
        self.button13.grid(row=4, column=9)

        self.button14 = tk.Button(root, text="REFRESH STATUSES", command=self.refresh_statuses)
        self.button14.grid(row=0, column=9)

        # get initial status of all variables from robot
        self.refresh_statuses()

    def send_Kp(self):
        self.post_request({'key': 'PID_Kp', 'value': self.entry_kp.get()})

    def send_Ki(self):
        self.post_request({'key': 'PID_Ki', 'value': self.entry_ki.get()})

    def send_Kd(self):
        self.post_request({'key': 'PID_Kd', 'value': self.entry_kd.get()})

    def send_setpoint(self):
        self.post_request({'key': 'PID_setpoint', 'value': deg_to_rad(float(self.entry_setpoint.get()))})
    
    def send_duty_cycle_min(self):
        self.post_request({'key': 'MOTOR_DUTY_CYCLE_MIN', 'value': self.entry_duty_cycle_min.get()})
    
    def send_duty_cycle_max(self):
        self.post_request({'key': 'MOTOR_DUTY_CYCLE_MAX', 'value': self.entry_duty_cycle_max.get()})
    
    def send_pitch_err_max(self):
        self.post_request({'key': 'PITCH_ANGLE_ERROR_MAX', 'value': deg_to_rad(float(self.entry_pitch_error_max.get()))})
    
    def send_pitch_err_min(self):
        self.post_request({'key': 'PITCH_ANGLE_ERROR_MIN', 'value': deg_to_rad(float(self.entry_pitch_error_min.get()))})
    
    def send_motor1_dir_manual(self):
        self.post_request({'key': 'MOTOR_1_DIR_MANUAL', 'value': self.entry_motor_1_dir_manual.get()})
    
    def send_motor2_dir_manual(self):
        self.post_request({'key': 'MOTOR_2_DIR_MANUAL', 'value': self.entry_motor_2_dir_manual.get()})
    
    def send_motor1_duty_cycle_manual(self):
        self.post_request({'key': 'MOTOR_1_DUTY_CYCLE_MANUAL', 'value': self.entry_motor_1_duty_cycle_manual.get()})
    
    def send_motor2_duty_cycle_manual(self):
        self.post_request({'key': 'MOTOR_2_DUTY_CYCLE_MANUAL', 'value': self.entry_motor_2_duty_cycle_manual.get()})
    
    def on_drive_mode_toggled(self):
        print('drive mode:', self.drive_mode.get())
        self.post_request({'key': 'CONTROL_MODE', 'value': self.drive_mode.get()})
    
    def on_motor_1_dir_toggled(self):
        self.post_request({'key': 'MOTOR_1_DIR_MANUAL', 'value': self.motor_1_dir_manual.get()})
    
    def on_motor_2_dir_toggled(self):
        self.post_request({'key': 'MOTOR_2_DIR_MANUAL', 'value': self.motor_2_dir_manual.get()})
    
    def send_stop(self):
        self.post_request({'key': 'EMERGENCY_STOP', 'value': ""})
        self.refresh_statuses()

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
    
    def get_request(self, url_suffix='status'):
        url = f'http://192.168.178.55/{url_suffix}'
        
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
    
    def refresh_statuses(self):
        status_info = self.get_request(url_suffix='status')

        # set initial values for each field
        writeEntry(self.entry_kp, status_info['PID_Kp'])
        writeEntry(self.entry_ki, status_info['PID_Ki'])
        writeEntry(self.entry_kd, status_info['PID_Kd'])
        writeEntry(self.entry_setpoint, rad_to_deg(status_info['PID_setpoint']))
        writeEntry(self.entry_duty_cycle_min, status_info['MOTOR_DUTY_CYCLE_MIN'])
        writeEntry(self.entry_duty_cycle_max, status_info['MOTOR_DUTY_CYCLE_MAX'])
        writeEntry(self.entry_pitch_error_max, rad_to_deg(status_info['PITCH_ANGLE_ERROR_MAX']))
        writeEntry(self.entry_pitch_error_min, rad_to_deg(status_info['PITCH_ANGLE_ERROR_MIN']))
        writeEntry(self.entry_motor_1_duty_cycle_manual, status_info['MOTOR_1_DUTY_CYCLE_MANUAL'])
        writeEntry(self.entry_motor_2_duty_cycle_manual, status_info['MOTOR_2_DUTY_CYCLE_MANUAL'])
        self.drive_mode.set(status_info['CONTROL_MODE'])
        self.motor_1_dir_manual.set(status_info['MOTOR_1_DIR_MANUAL'])
        self.motor_2_dir_manual.set(status_info['MOTOR_2_DIR_MANUAL'])

        self.entry_pitch_current.configure(state='normal')
        writeEntry(self.entry_pitch_current, rad_to_deg(status_info['pitch_angle_current']))
        self.entry_pitch_current.configure(state='readonly')


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()

