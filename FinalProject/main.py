""""
Project 3: Motor Control with PWM
This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)

Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
"""

import time
from motor import MotorController
from encoder import MotorEncoder

if __name__ == "__main__":
    motor_ctrl = MotorController()
    motor_enc = MotorEncoder()

    try:
        while True:
            # 1. Wait a consistent amount of time (20Hz - 50Hz)
            time.sleep(0.05) 
            
            # 2. Measure and Update
            rpm = motor_enc.calculate_rpm()
            pwm = motor_ctrl.update_speed(120, rpm)
            motor_ctrl.set_speed(pwm)
        
    except KeyboardInterrupt:
        motor_ctrl.plot_speed_PID()

    # Terminate motor controller and clean up GPIO settings
    motor_ctrl.stop()