import matplotlib
# Use the 'Agg' backend to avoid memory-heavy GUI windows
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import time
from time import sleep

# Minimum DC PWM
min_speed_pwm = 7.5
max_speed_pwm = 8.2
default_speed_pwm = 8.1
steering_forward = 7.5 
steering_right = 9.0
steering_left = 6.0 

# PID Controller
speed_Kp = 0.001
speed_Ki = 0.005
speed_Kd = 0.00003


# Plot for Tuning PID
speed_plot = {
    "rpm_target"  : [],
    "rpm_err"     : [],
    "rpm_actual"  : [],
    "p"           : [], 
    "i"           : [],
    "d"           : [],
}

"""
    Control Motor
"""
class MotorController:
    def __init__(self, speed_pin=18, steering_pin=19):
        self.speed_pin = speed_pin
        self.steering_pin = steering_pin

        # PID Controller Init
        self.pid_speed_integral = 0
        self.pid_speed_last_error = 0
        self.last_time = time.time()
        
        # Pin Initialization
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.speed_pin, GPIO.OUT)
        GPIO.setup(self.steering_pin, GPIO.OUT)

        self.speed_pwm = GPIO.PWM(self.speed_pin, 50) # set the frequency to 50 Hz 
        self.steering_pwm = GPIO.PWM(self.steering_pin, 50) # set the frequency to 50 Hz 

        self.speed_pwm.start(7.5) # Don't change the 7.5 DC because it is for initialization
        # sleep(0.5)
        # self.speed_pwm.start(10.0) # Don't change the 7.5 DC because it is for initialization
        # sleep(0.1)
        # self.speed_pwm.start(5.0) # Don't change the 7.5 DC because it is for initialization
        # sleep(0.5)
        # self.speed_pwm.start(7.5) # Don't change the 7.5 DC because it is for initialization
        self.steering_pwm.start(steering_forward) # Start with the specified duty cycle

    def set_speed(self, duty_cycle):
        self.speed_pwm.ChangeDutyCycle(duty_cycle)

    def set_steering(self, direction="forward"):
        if direction == "forward":
            self.steering_pwm.ChangeDutyCycle(steering_forward)
        elif direction == "right":
            self.steering_pwm.ChangeDutyCycle(steering_right)
        elif direction == "left":
            self.steering_pwm.ChangeDutyCycle(steering_left)

    def stop(self):
        self.set_speed(0)
        self.speed_pwm.stop()
        self.steering_pwm.stop()
        GPIO.cleanup()

    def update_speed(self, target_RPM, measured_RPM):
        current_time = time.time()
        dt = current_time - self.last_time

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

        print(f"Measured RPM: {measured_RPM:6.1f} | Error RPM: {error:6.1f} --- PID({_P:1.3f}, {_I:1.3f}, {_D:1.3f}) -> PWM: {final_pwm:5.3f}%")

        # For tuning the PID
        speed_plot["rpm_target"].append(target_RPM)
        speed_plot["rpm_actual"].append(measured_RPM)
        speed_plot["rpm_err"].append(error) 
        speed_plot["p"].append(_P) 
        speed_plot["i"].append(_I)
        speed_plot["d"].append(_D)

        # Update state
        self.last_time = current_time
        self.pid_speed_last_error = error

        return final_pwm

    def plot_speed_PID(self):
        # --- Plotting the Results ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        # Top Plot: RPM Performance
        ax1.plot(speed_plot["rpm_target"], label="Target RPM", linestyle="--")
        ax1.plot(speed_plot["rpm_actual"], label="Actual RPM", linewidth=2)
        ax1.plot(speed_plot["rpm_err"], label="Error RPM", linestyle=":")
        ax1.set_ylabel("Speed (RPM)")
        ax1.legend()
        ax1.set_title("PID Tuning Response")

        # Bottom Plot: PID Term Contributions
        ax2.plot(speed_plot["p"], label="P (Proportional)")
        ax2.plot(speed_plot["i"], label="I (Integral)")
        ax2.plot(speed_plot["d"], label="D (Derivative)")
        ax2.set_ylabel("PWM Correction Value")
        ax2.set_xlabel("Sample Number")
        ax2.legend()

        plt.tight_layout()
        plt.savefig("PID_tuning.png", dpi=150)
        
        # CRITICAL: Close the plot to free memory
        plt.close(fig)


# if __name__ == "__main__":
#     motor_controller = MotorController() # Initialize motor controller with default pins and steering direction
#     try:
#         while True:
#             # Move forward
#             motor_controller.set_steering("forward")
#             motor_controller.set_speed(default_speed_pwm)
#             sleep(0.5)
            
#             # Stop
#             motor_controller.set_speed(0)
#             sleep(0.5)
            
#             # Turn left
#             motor_controller.set_steering("left")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(1)
            
#             # Turn right
#             motor_controller.set_steering("right")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(1)
            
#             # Turn straight
#             motor_controller.set_steering("left")
#             motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
#             sleep(0.5)
#     except KeyboardInterrupt:
#         pass

#     # Terminate motor controller and clean up GPIO settings
#     motor_controller.stop()