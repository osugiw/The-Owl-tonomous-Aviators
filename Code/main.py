#""""
#Project 3: Motor Control with PWM
#This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)
#
#Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
#"""
#
#import time
#import cv2
#from motor import MotorController
#from encoder import MotorEncoder
#from poly_lane2 import LaneTracker as PolyLaneTracker
#
#RED_THR = 12000
#
#if __name__ == "__main__":
#    motor_ctrl = MotorController()
#    motor_enc = MotorEncoder()
#    poly_lane_detect = PolyLaneTracker()
#    red_cnt = 0
#    red_flag = False
#    try:
#        while True:
#            # 1. Wait a consistent amount of time (20Hz - 50Hz)
#            time.sleep(0.05) 
#            angle, red, img = poly_lane_detect.get_lane_data()
#            steering_error = abs(angle - 90) if angle is not None else 0
#
#            if steering_error > 25:
#                target_rpm = 20    # sharp curve
#            elif steering_error > 10:
#                target_rpm = 30    # mild curve
#            else:
#                target_rpm = 60   # straight
#
#            steering_pwm = motor_ctrl.update_steering(90, angle)
#            motor_ctrl.set_steering_pid(steering_pwm)      
#            if img is not None:
#                cv2.imshow("Perfected View", img)
#                if cv2.waitKey(1) & 0xFF == ord('q'):
#                  break
#            
##            # Red Box detection
##            if red is not None  and red_flag == False and red > RED_THR:
##                red_cnt += 1
##                red_flag = True
##                print(f"Red Box Detected! (Size: {red}). Stopping the car! - Box count {red_cnt}")
##                
##                # Move the robot a little bit
##                motor_ctrl.set_speed(8.0)   
##                time.sleep(3)
##
##                # First stop
##                if red_cnt == 1 and red_flag == True:
##                    print("First stop detected!")
##                    motor_ctrl.set_speed(0)   
##                    motor_ctrl.set_steering(7.5)
##                    time.sleep(5)
##                    red_flag = False
##                elif red_cnt == 2 and red_flag == True:
##                    print("Second stop detected!")
##                    break
#
#            # Control speed
#            rpm = motor_enc.calculate_rpm()
#            speed_pwm = motor_ctrl.update_speed(80, rpm)
#            motor_ctrl.set_steering_pid(new_pwm)
#            motor_ctrl.set_speed(speed_pwm)  
#            time.sleep(2)  
#
#    except KeyboardInterrupt:
#      print("Terminating robot")
#      pass
#    
#    finally:
#      # Clear the process     
#      motor_ctrl.stop()
#      poly_lane_detect.clean_up()
#      # hough_lane_detect.clean_up()
#      motor_ctrl.plot_steering_PID()
#      motor_ctrl.plot_speed_PID()
#      print("Robot terminated...")

#""""
#Project 3: Motor Control with PWM
#This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)
#
#Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
#"""
#
#import time
#import cv2
#from motor import MotorController
#from encoder import MotorEncoder
#from poly_lane2 import LaneTracker as PolyLaneTracker
#
#if __name__ == "__main__":
#    motor_ctrl = MotorController()
#    motor_enc = MotorEncoder()
#    poly_lane_detect = PolyLaneTracker()
#    count_frame = 0
#    red_cnt = 0
#    red_flag = False
#    try:
#        while True:
#            # 1. Wait a consistent amount of time (20Hz - 50Hz)
#            actual_steering, red, img = poly_lane_detect.get_lane_data()
#            count_frame += 1
#            if img is not None:
#                cv2.imshow("Perfected View", img)
#                if cv2.waitKey(1) & 0xFF == ord('q'):
#                    break
#            
##            # Red Box detection
##            if count_frame % 3 == 0:
##
##                count_frame = 0 
##                
##                if red is not None and red > 12000 and red_flag == False:
##                    red_cnt += 1
##                    red_flag = True
##                    
##                    print(f"Red Box Detected! (Size: {red}). Stopping the car! - Box count {red_cnt}")
##                    
##                    # First stop
##                    if red_cnt == 1:
##                        print("First stop detected!")
##                        motor_ctrl.set_speed(0)   
##                        motor_ctrl.set_steering(7.5)
##                        time.sleep(5)
##
##                        # Move the robot a little bit
##                        motor_ctrl.set_speed(8.1)   
##                        time.sleep(2)
##
##                    elif red_cnt == 2:
##                        print("Second stop detected!")
##                        motor_ctrl.stop()
##                        break
##
##                if (red is None or red <= 12000) and red_flag == True:
##                    print("Red box disappeared, ready for next detection")
##                    red_flag = False
#                
#            # Steering control
#            steering_pwm = motor_ctrl.update_steering(90, actual_steering)
#            motor_ctrl.set_steering_pid(steering_pwm)
#            
#            # Adjust the speed based on steering error
#            steering_error = abs(actual_steering - 90)
#            if steering_error > 15:
#                target_rpm = 35  # Sharp turn
#            elif steering_error > 8:
#                target_rpm = 42  # Moderate turn
#            else:
#                target_rpm = 55
#            rpm = motor_enc.calculate_rpm()
#            speed_pwm = motor_ctrl.update_speed(target_rpm, rpm)
##            motor_ctrl.set_speed(speed_pwm)   
#            motor_ctrl.set_speed(0)
#
#    except KeyboardInterrupt:
#        pass
#        
#    motor_ctrl.stop()
#    poly_lane_detect.clean_up()
#    motor_ctrl.plot_speed_PID()
#    motor_ctrl.plot_steering_PID()
#    print("Robot terminated...")


#"""
#Project 3: Motor Control with PWM
#This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)
#
#Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
#"""
#
#import time
#import cv2
#from motor import MotorController
#from encoder import MotorEncoder
#from poly_lane2 import LaneTracker as PolyLaneTracker
#
#if __name__ == "__main__":
#    motor_ctrl = MotorController()
#    motor_enc = MotorEncoder()
#    poly_lane_detect = PolyLaneTracker()
#
#    count_frame = 0
#    red_cnt = 0
#    red_flag = False
#
#    prev_steering = 90
#    pred_alpha = 0.3        
#    try:
#        while True:
#            time.sleep(0.02)   
#            actual_steering, red, img = poly_lane_detect.get_lane_data()
#            count_frame += 1
#
#            if img is not None:
#                cv2.imshow("Perfected View", img)
#                if cv2.waitKey(1) & 0xFF == ord('q'):
#                    break
#
#            # Red box detection
#            if count_frame % 3 == 0:
#                count_frame = 0
#
#                if red is not None and red > 12000 and not red_flag:
#                    red_cnt += 1
#                    red_flag = True
#
#                    print(f"Red Box Detected! (Size: {red}). Stopping the car! - Box count {red_cnt}")
#
#                    if red_cnt == 1:
#                        print("First stop detected!")
#                        motor_ctrl.set_speed(0)
#                        motor_ctrl.set_steering_pid(7.5)
#                        time.sleep(5)
#                        motor_ctrl.set_speed(8.1)
#                        time.sleep(2)
#
#                    elif red_cnt == 2:
#                        print("Second stop detected!")
#                        motor_ctrl.stop()
#                        break
#
#                if (red is None or red <= 12000) and red_flag:
#                    print("Red box disappeared, ready for next detection")
#                    red_flag = False
#
#
#            if actual_steering is not None:
#                delta = actual_steering - prev_steering
#            
#                # If angle is suspiciously far from straight, dampen it
#                if abs(actual_steering - 90) > 15:
#                    print(f"[Warning] Steering angle {actual_steering:.1f} seems extreme, dampening")
#                    actual_steering = prev_steering + (0.5 * delta)  # don't trust it fully
#            
#                predicted_steering = actual_steering + (pred_alpha * delta)
#                predicted_steering = max(50, min(130, predicted_steering))
#                prev_steering = actual_steering
#            else:
#                predicted_steering = prev_steering
#
#  
#            if actual_steering is not None and (actual_steering < 75 or actual_steering > 105):
#                target_rpm = 40
#            else:
#                target_rpm = 50
#
#
#            rpm = motor_enc.calculate_rpm()
#            speed_pwm = motor_ctrl.update_speed(target_rpm, rpm)
#            motor_ctrl.set_speed(speed_pwm)
##            motor_ctrl.set_speed(0)
#
#    except KeyboardInterrupt:
#        pass
#
#    motor_ctrl.stop()
#    poly_lane_detect.clean_up()
#    motor_ctrl.plot_speed_PID()
#    motor_ctrl.plot_steering_PID()
#    print("Robot terminated")


"""
Project 3: Motor Control with PWM
This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)

Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
"""

import time
import cv2
from motor import MotorController
from encoder import MotorEncoder
from poly_lane2 import LaneTracker as PolyLaneTracker

if __name__ == "__main__":
    motor_ctrl = MotorController()
    motor_enc = MotorEncoder()
    poly_lane_detect = PolyLaneTracker()

    count_frame = 0
    red_cnt = 0
    red_flag = False

    prev_steering = 90
    pred_alpha = 0.3

    try:
        while True:
            time.sleep(0.02)
            actual_steering, red, img = poly_lane_detect.get_lane_data()
            count_frame += 1

            if img is not None:
                cv2.imshow("Perfected View", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if count_frame % 3 == 0:
                count_frame = 0

                if red is not None and red > 12000 and not red_flag:
                    red_cnt += 1
                    red_flag = True

                    print(f"Red Box Detected! (Size: {red}). Stopping the car! - Box count {red_cnt}")

                    if red_cnt == 1:
                        print("First stop detected!")
                        motor_ctrl.set_speed(0)
                        motor_ctrl.set_steering_pid(7.5)
                        time.sleep(5)
                        motor_ctrl.set_speed(8.1)
                        time.sleep(2)

                    elif red_cnt == 2:
                        print("Second stop detected!")
                        motor_ctrl.stop()
                        break

                if (red is None or red <= 12000) and red_flag:
                    print("Red box disappeared, ready for next detection")
                    red_flag = False
                    
            if actual_steering is not None:
                delta = actual_steering - prev_steering

                if abs(actual_steering - 90) > 35:
                    print(f"[Warning] Extreme angle {actual_steering:.1f}, dampening")
                    actual_steering = prev_steering + (0.5 * delta)

                predicted_steering = actual_steering + (pred_alpha * delta)
                predicted_steering = max(50, min(130, predicted_steering))
                prev_steering = actual_steering
            else:
                predicted_steering = prev_steering

            print(f"Steering: {actual_steering} | Predicted: {predicted_steering:.1f}", end="\r")

            steering_pwm = motor_ctrl.update_steering(90, predicted_steering)
            motor_ctrl.set_steering_pid(steering_pwm)

            if actual_steering is not None and (actual_steering < 75 or actual_steering > 105):
                target_rpm = 50
            else:
                target_rpm = 60

            rpm = motor_enc.calculate_rpm()
            speed_pwm = motor_ctrl.update_speed(target_rpm, rpm)
            motor_ctrl.set_speed(speed_pwm)

    except KeyboardInterrupt:
        pass

    motor_ctrl.stop()
    poly_lane_detect.clean_up()
    motor_ctrl.plot_speed_PID()
    motor_ctrl.plot_steering_PID()
    print("\nRobot terminated")