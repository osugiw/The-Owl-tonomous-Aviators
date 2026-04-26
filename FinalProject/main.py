""""
Project 3: Motor Control with PWM
This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)

Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
"""

import time
import cv2
from motor import MotorController
from encoder import MotorEncoder
from poly_lane2 import LaneTracker as PolyLaneTracker
from hough_lane import LaneTracker as HoughLaneTracker

if __name__ == "__main__":
    motor_ctrl = MotorController()
    motor_enc = MotorEncoder()
    poly_lane_detect = PolyLaneTracker()
    # hough_lane_detect = HoughLaneTracker()
    red_cnt = 0
    try:
        while True:
            # 1. Wait a consistent amount of time (20Hz - 50Hz)
            time.sleep(0.05) 
            actual_steering, red, img = poly_lane_detect.get_lane_data()
            # actual_steering, red, img = hough_lane_detect.get_lane_data()
            if img is not None:
                cv2.imshow("Perfected View", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Red Box detection
            if red is not None and red > 50000 and red < 110000:
                red_cnt++
                print(f"Red Box Detected! (Size: {red}). Stopping the car!")
                # time.sleep(1)
                motor_ctrl.set_speed(0)          # Cut power to the drive motor
                motor_ctrl.set_steering(7.5)     # (Optional) Straighten the wheels
                time.sleep(1)
                
                # Stop the program
                if red_cnt == 2:
                    break

            # Steering control
            steering_pwm = motor_ctrl.update_steering(90, actual_steering)
            motor_ctrl.set_steering(steering_pwm)
            time.sleep(0.05)

            # Control speed
            rpm = motor_enc.calculate_rpm()
            speed_pwm = motor_ctrl.update_speed(50, rpm)
            motor_ctrl.set_speed(speed_pwm)       

    except KeyboardInterrupt:
        motor_ctrl.stop()
        poly_lane_detect.clean_up()
        # hough_lane_detect.clean_up()
        motor_ctrl.plot_speed_PID()
        motor_ctrl.plot_steering_PID()
        print("Robot terminated...")