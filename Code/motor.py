import matplotlib
# Use the 'Agg' backend to avoid memory-heavy GUI windows
matplotlib.use('Agg') 
import numpy as np
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
from time import sleep

# Minimum DC PWM
min_speed_pwm = 7.5
max_speed_pwm = 8.2
default_speed_pwm = 7.85
steering_forward = 7.5 
steering_right = 9.0
steering_left = 6.0 

steering_trim = 2

# PID Controller
speed_Kp = 0.015
speed_Ki = 0.0072
speed_Kd = 0.0017
#speed_Kp = 0.015
#speed_Ki = 0.0072
#speed_Kd = 0.0017

steering_Kp = 0.1
steering_Ki = 0.00015
steering_Kd = 0.0065
#steering_Kp = 0.1
#steering_Ki = 0.0
#steering_Kd = 0.00659

# Plot for Tuning PID
speed_plot = {
    "rpm_target"  : [],
    "rpm_err"     : [],
    "rpm_actual"  : [],
    "p"           : [], 
    "i"           : [],
    "d"           : [],
    "pwm"         : [],
}
steering_plot = {
    "steering_target"  : [],
    "steering_err"     : [],
    "steering_actual"  : [],
    "p"           : [], 
    "i"           : [],
    "d"           : [],
    "pwm"         : [],
}

speed_pin = 18
steering_pin = 19
"""
    Control Motor
"""
class MotorController:
    def __init__(self):
        self.last_steering_pwm = 7.5

        # PID Controller Init
        self.pid_speed_integral = 0
        self.pid_speed_last_error = 0
        self.pid_speed_last_time = time.time()
        self.pid_steering_integral = 0
        self.pid_steering_last_error = 0
        self.pid_steering_last_time = time.time()
        
        # Pin Initialization
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(speed_pin, GPIO.OUT)
        GPIO.setup(steering_pin, GPIO.OUT)
        self.speed_pwm = GPIO.PWM(speed_pin, 50) # set the frequency to 50 Hz 
        self.steering_pwm = GPIO.PWM(steering_pin, 50) # set the frequency to 50 Hz 
        
        # Initialization
        self.speed_pwm.start(7.5) # Don't change the 7.5 DC because it is for initialization
        self.steering_pwm.start(steering_forward) # Start with the specified duty cycle
        sleep(2)
        
#         Speed init if needed
#        self.speed_pwm.start(10.0) # Don't change the 7.5 DC because it is for initialization
#        sleep(2)
#        self.speed_pwm.start(5.0) # Don't change the 7.5 DC because it is for initialization
#        sleep(2)
#        self.speed_pwm.start(7.5) # Don't change the 7.5 DC because it is for initialization
        
    def set_speed(self, duty_cycle):
        self.speed_pwm.ChangeDutyCycle(duty_cycle)

    def set_steering(self, angle="forward"):
        if angle == "forward":
            self.steering_pwm.ChangeDutyCycle(steering_forward)
        elif angle == "right":
            self.steering_pwm.ChangeDutyCycle(steering_right)
        elif angle == "left":
            self.steering_pwm.ChangeDutyCycle(steering_left)

    def set_steering_pid(self, pwm):
        self.steering_pwm.ChangeDutyCycle(pwm)

    def stop(self):
        self.set_speed(7.5)
        self.set_steering_pid(7.5)

    def update_speed(self, target_RPM, measured_RPM):
        current_time = time.time()
        dt = current_time - self.pid_speed_last_time

        # Prevent zero DC
        if dt < 0.010:
            return 7.5
        
        # Calculate the difference between target and measured
        error = target_RPM - measured_RPM
        
        # PID Calculation
        _P = speed_Kp * error
        self.pid_speed_integral += error * dt   # Accumulate error overtime for calculating Integral part
        _I = speed_Ki * self.pid_speed_integral
        _D =  speed_Kd * ((error - self.pid_speed_last_error) / dt)
        
        # New adjustment
        output = min_speed_pwm + (_P + _I + _D)
        final_pwm = max(min_speed_pwm, min(max_speed_pwm, output))  # ADC Mapping for final Duty Cycle PWM

#        print(f"[Speed - RPM] Measured: {measured_RPM:6.1f} | Error: {error:6.1f} --- PID({_P:1.3f}, {_I:1.3f}, {_D:1.3f}) -> PWM: {final_pwm:5.3f}%")

        # For tuning the PID
        speed_plot["rpm_target"].append(target_RPM)
        speed_plot["rpm_actual"].append(measured_RPM)
        speed_plot["rpm_err"].append(error) 
        speed_plot["p"].append(_P) 
        speed_plot["i"].append(_I)
        speed_plot["d"].append(_D)
        speed_plot["pwm"].append(final_pwm)

        # Update state
        self.pid_speed_last_time = current_time
        self.pid_speed_last_error = error
        return final_pwm

    def update_steering(self, target_angle, actual_angle):
        if actual_angle is None:
            return self.last_steering_pwm  # Neutral/Straight

        now = time.time()
        dt = now - self.pid_steering_last_time
        
        if dt < 0.01: 
          return self.last_steering_pwm

        # Calculate Error
        error = (actual_angle - target_angle)
        
        if(abs(error) < 2):
          error = 0
        
        # PID Calculation
        _P = steering_Kp * error
        self.pid_steering_integral += error * dt
        self.pid_steering_integral = max(-20, min(20, self.pid_steering_integral))  # anti-windup clamp
        _I = steering_Ki * self.pid_steering_integral
        _D = steering_Kd * (error - self.pid_steering_last_error) / dt
        raw_correction = _P + _I + _D

        # Apply Sensitivity and Neutral Offset
        final_pwm = 7.5 + (raw_correction * 2.6)
        
        # Final Hardware Clamp
        final_pwm = max(steering_left, min(steering_right, final_pwm))

        # Save data for your plots
        steering_plot["steering_target"].append(target_angle)
        steering_plot["steering_actual"].append(actual_angle)
        steering_plot["steering_err"].append(error) 
        steering_plot["p"].append(_P)
        steering_plot["i"].append(_I) # We are using PD now, so I is 0
        steering_plot["d"].append(_D)
        steering_plot["pwm"].append(final_pwm)

#        print(f"[Steering - Degree] Measured: {actual_angle:6.1f} | Error: {error:6.1f} --- PID({_P:1.3f}, {_I:1.3f}, {_D:1.3f}) -> PWM: {final_pwm:5.3f}%")

        self.pid_steering_last_time = now
        self.pid_steering_last_error = error
        self.last_steering_pwm = final_pwm
        return final_pwm
    
    def save_pid_plot(self, data_dict, filename, title, y_label_top):
        """Helper function to handle all PID plotting logic."""
        # Check for data and synchronized keys
        if not data_dict or len(next(iter(data_dict.values()))) < 2:
            print(f"Not enough data to plot {title}.")
            return

        try:
            # Sync lengths and create snapshots
            min_len = min(len(data_dict[k]) for k in data_dict.keys())
            
            # Map the specific keys for speed/steering dynamically
            p_val = np.array(data_dict["p"][:min_len])
            i_val = np.array(data_dict["i"][:min_len])
            d_val = np.array(data_dict["d"][:min_len])
            
            # Identify which keys to use for the top plot
            k_target = [k for k in data_dict if "target" in k][0]
            k_actual = [k for k in data_dict if "actual" in k][0]
            k_err    = [k for k in data_dict if "err" in k][0]

            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

            # Top Plot: Performance
            ax1.plot(np.array(data_dict[k_target][:min_len]), label="Target", linestyle="--", alpha=0.8)
            ax1.plot(np.array(data_dict[k_actual][:min_len]), label="Actual", linewidth=2)
            ax1.plot(np.array(data_dict[k_err][:min_len]), label="Error", linestyle=":", color='gray')
            ax1.set_ylabel(y_label_top)
            ax1.set_title(title)
            ax1.legend(loc='upper right')

            # Bottom Plot: PID Terms
            ax2.plot(p_val, label="P (Proportional)")
            ax2.plot(i_val, label="I (Integral)")
            ax2.plot(d_val, label="D (Derivative)")
            ax2.set_ylabel("Correction (PWM)")
            ax2.set_xlabel("Sample Number")
            ax2.legend(loc='upper right')

            plt.subplots_adjust(hspace=0.3) 
            plt.savefig(filename, dpi=100)
            plt.close(fig)
            print(f"Successfully saved: {filename}")

        except Exception as e:
            print(f"Failed to plot {title}: {e}")
            
    def plot_combined_run(self, filename="DutyCycle_SpeedSteering.png"):
        """Plots Error, Steering PWM, and Speed PWM on a single unified plot."""
        # Ensure we have the 'pwm' keys in our dictionaries
        if "pwm" not in steering_plot or "pwm" not in speed_plot:
            print("Error: 'pwm' data not found in plot dictionaries.")
            return

        if not steering_plot["steering_err"] or not speed_plot["pwm"]:
            print("Not enough data for combined plot.")
            return

        try:
            # Sync lengths
            min_len = min(len(steering_plot["steering_err"]), 
                          len(steering_plot["pwm"]), 
                          len(speed_plot["pwm"]))
            
            frames = np.arange(min_len)
            error_data = np.array(steering_plot["steering_err"][:min_len])
            steer_pwm_data = np.array(steering_plot["pwm"][:min_len])
            speed_pwm_data = np.array(speed_plot["pwm"][:min_len])

            fig, ax1 = plt.subplots(figsize=(12, 7))

            # --- Primary Axis: Error (Degrees) ---
            lns1 = ax1.plot(frames, error_data, color='red', label="Steering Error (Deg)", alpha=0.7)
            ax1.axhline(0, color='red', linestyle='--', alpha=0.3)
            ax1.set_xlabel("Frame Number")
            ax1.set_ylabel("Error (Degrees)", color='red')
            ax1.tick_params(axis='y', labelcolor='red')

            # --- Secondary Axis: Duty Cycles (%) ---
            ax2 = ax1.twinx()
            lns2 = ax2.plot(frames, steer_pwm_data, color='blue', label="Steering PWM (%)", linewidth=2)
            lns3 = ax2.plot(frames, speed_pwm_data, color='green', label="Speed PWM (%)", linewidth=2)
            ax2.axhline(7.5, color='blue', linestyle=':', alpha=0.3) # Steering Neutral Reference
            ax2.set_ylabel("Duty Cycle (%)", color='black')
            
            # Add a title and adjust layout
            plt.title("Combined Performance: Error vs Duty Cycles")
            
            # Combine legends from both axes into one box
            lns = lns1 + lns2 + lns3
            labs = [l.get_label() for l in lns]
            ax1.legend(lns, labs, loc='upper right', framealpha=0.9)

            fig.tight_layout()
            plt.savefig(filename, dpi=150)
            plt.close(fig)
            print(f"Successfully saved single-frame combined plot: {filename}")

        except Exception as e:
            print(f"Failed to generate combined plot: {e}")

    # --- Simplified Callers ---
    def plot_speed_PID(self):
        self.save_pid_plot(speed_plot, "PID_speed.png", "Speed PID Response", "RPM")

    def plot_steering_PID(self):
        self.save_pid_plot(steering_plot, "PID_steering.png", "Steering PID Response", "Degrees")


# if __name__ == "__main__":
#     motor_controller = MotorController() # Initialize motor controller with default pins and steering angle
#     try:
#         while True:
#             # Move forward
#             motor_controller.set_steering("forward")
#             motor_controller.set_speed(default_speed_pwm)
#             sleep(1)
        
#             # Stop
#             motor_controller.set_speed(0)
#             sleep(0.5)
        
#             # Turn left
#             motor_controller.set_steering("left")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(2)
        
#             # Turn right
#             motor_controller.set_steering("right")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(2)
        
#             # Turn straight
#             motor_controller.set_steering("left")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(1)
#     except KeyboardInterrupt:
#         # Terminate motor controller and clean up GPIO settings
#         motor_controller.stop()
#         pass

    
