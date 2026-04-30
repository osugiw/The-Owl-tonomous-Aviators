
"""
@Description: 
    This project was developed as the final project for ELEC 553. The objective was to design and implement an autonomous vehicle capable of navigating a predefined track using computer vision and control techniques.
@Contributor: 
    Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
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

    red_box_thr = 2500

    try:
        while True:
            # Process the land detection and robot steering angle
            time.sleep(0.02)
            actual_steering, red, img = poly_lane_detect.get_lane_data()
            count_frame += 1
            
            if img is not None:
                cv2.imshow("Perfected View", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # Box red detection
            if count_frame % 3 == 0:
                count_frame = 0
                if red is not None and red > red_box_thr and not red_flag:
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

                if (red is None or red <= red_box_thr) and red_flag:
                    print("Red box disappeared, ready for next detection")
                    red_flag = False

            # Steering angle calculation and update
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

            # Update the speed based on calcualted steering angle
            if actual_steering is not None and (actual_steering < 75 or actual_steering > 105):
                target_rpm = 30
            else:
                target_rpm = 40
            rpm = motor_enc.calculate_rpm()
            speed_pwm = motor_ctrl.update_speed(target_rpm, rpm)
            motor_ctrl.set_speed(speed_pwm)
    except KeyboardInterrupt:
        pass

    # Clear the pipeline and GPIOs
    motor_ctrl.stop()
    poly_lane_detect.clean_up()

    # Plot PID and data for debugging
    motor_ctrl.plot_speed_PID()
    motor_ctrl.plot_steering_PID()
    motor_ctrl.plot_combined_run()
    print("\nRobot terminated")