import tkinter as tk
import requests

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("REST API Client")

        # input fields
        self.field1_label = tk.Label(root, text="Kp")
        self.field1_label.grid(row=0, column=0)
        self.field1_entry = tk.Entry(root)
        self.field1_entry.grid(row=0, column=1)

        self.field2_label = tk.Label(root, text="Ki")
        self.field2_label.grid(row=1, column=0)
        self.field2_entry = tk.Entry(root)
        self.field2_entry.grid(row=1, column=1)

        self.field3_label = tk.Label(root, text="Kd")
        self.field3_label.grid(row=2, column=0)
        self.field3_entry = tk.Entry(root)
        self.field3_entry.grid(row=2, column=1)

        self.field4_label = tk.Label(root, text="Setpoint")
        self.field4_label.grid(row=3, column=0)
        self.field4_entry = tk.Entry(root)
        self.field4_entry.grid(row=3, column=1)

        self.field5_label = tk.Label(root, text="MOTOR_DUTY_CYCLE_MIN")
        self.field5_label.grid(row=0, column=3)
        self.field5_entry = tk.Entry(root)
        self.field5_entry.grid(row=0, column=4)

        self.field6_label = tk.Label(root, text="MOTOR_DUTY_CYCLE_MAX")
        self.field6_label.grid(row=1, column=3)
        self.field6_entry = tk.Entry(root)
        self.field6_entry.grid(row=1, column=4)

        self.field7_label = tk.Label(root, text="PITCH_ANGLE_ERROR_MAX")
        self.field7_label.grid(row=2, column=3)
        self.field7_entry = tk.Entry(root)
        self.field7_entry.grid(row=2, column=4)

        self.field8_label = tk.Label(root, text="PITCH_ANGLE_ERROR_MIN")
        self.field8_label.grid(row=3, column=3)
        self.field8_entry = tk.Entry(root)
        self.field8_entry.grid(row=3, column=4)

        self.field9_label = tk.Label(root, text="MOTOR_1_DIR_MANUAL")
        self.field9_label.grid(row=0, column=6)
        self.field9_entry = tk.Entry(root)
        self.field9_entry.grid(row=0, column=7)

        self.field10_label = tk.Label(root, text="MOTOR_2_DIR_MANUAL")
        self.field10_label.grid(row=1, column=6)
        self.field10_entry = tk.Entry(root)
        self.field10_entry.grid(row=1, column=7)

        self.field11_label = tk.Label(root, text="MOTOR_1_DUTY_CYCLE_MANUAL")
        self.field11_label.grid(row=2, column=6)
        self.field11_entry = tk.Entry(root)
        self.field11_entry.grid(row=2, column=7)

        self.field12_label = tk.Label(root, text="MOTOR_2_DUTY_CYCLE_MANUAL")
        self.field12_label.grid(row=3, column=6)
        self.field12_entry = tk.Entry(root)
        self.field12_entry.grid(row=3, column=7)

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

    def send_Kp(self):
        self.send_request({'key': 'PID_Kp', 'value': self.field1_entry.get()})

    def send_Ki(self):
        self.send_request({'key': 'PID_Ki', 'value': self.field2_entry.get()})

    def send_Kd(self):
        self.send_request({'key': 'PID_Kd', 'value': self.field3_entry.get()})

    def send_setpoint(self):
        self.send_request({'key': 'PID_setpoint', 'value': self.field4_entry.get()})
    
    def send_duty_cycle_min(self):
        self.send_request({'key': 'MOTOR_DUTY_CYCLE_MIN', 'value': self.field5_entry.get()})
    
    def send_duty_cycle_max(self):
        self.send_request({'key': 'MOTOR_DUTY_CYCLE_MAX', 'value': self.field6_entry.get()})
    
    def send_pitch_err_max(self):
        self.send_request({'key': 'PITCH_ANGLE_ERROR_MAX', 'value': self.field7_entry.get()})
    
    def send_pitch_err_min(self):
        self.send_request({'key': 'PITCH_ANGLE_ERROR_MIN', 'value': self.field8_entry.get()})
    
    def send_motor1_dir_manual(self):
        self.send_request({'key': 'MOTOR_1_DIR_MANUAL', 'value': self.field9_entry.get()})
    
    def send_motor2_dir_manual(self):
        self.send_request({'key': 'MOTOR_2_DIR_MANUAL', 'value': self.field10_entry.get()})
    
    def send_motor1_duty_cycle_manual(self):
        self.send_request({'key': 'MOTOR_1_DUTY_CYCLE_MANUAL', 'value': self.field11_entry.get()})
    
    def send_motor2_duty_cycle_manual(self):
        self.send_request({'key': 'MOTOR_2_DUTY_CYCLE_MANUAL', 'value': self.field12_entry.get()})

    def send_request(self, data):
        url = 'http://192.168.178.55/set-value'  # Replace with your actual server URL and endpoint
        
        print('Sending request with body:', data)
        
        try:
            # response = requests.post(url, json=data, headers=headers)
            response = requests.post(url, data=data)
            if response.status_code == 200:
                print(f"Request successful: {response.text}")
                # Optionally, show a message or handle success
            else:
                print(f"Request failed with status code: {response.status_code}: {response.text}")
                # Optionally, show an error message or handle failure
        except requests.exceptions.RequestException as e:
            print(f"Request failed: {e}")
            # Handle exceptions, e.g., connection errors

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()

