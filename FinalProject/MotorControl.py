""""
Project 3: Motor Control with PWM
This project demonstrates how to control a motor's speed and steering using Pulse Width Modulation (PWM)

Teams: Ashley Garcia, Mehul Goel, Sugiarto Wibowo, Valeria Itrat Herrera
"""

import RPi.GPIO as GPIO
from time import sleep

# Minimum DC PWM
default_speed_pwm = 8.1
steering_forward = 7.5 
steering_right = 9.0
steering_left = 6.0 


class MotorController:
    def __init__(self, speed_pin=18, steering_pin=19):
        self.speed_pin = speed_pin
        self.steering_pin = steering_pin
        
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



if __name__ == "__main__":
    motor_controller = MotorController() # Initialize motor controller with default pins and steering direction

    try:
        while True:
            # Move forward
            motor_controller.set_steering("forward")
            motor_controller.set_speed(default_speed_pwm)
            sleep(0.5)
            
            # Stop
            motor_controller.set_speed(0)
            sleep(0.5)
            
            # Turn left
            motor_controller.set_steering("left")
            motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
            sleep(1)
            
            # Turn right
            motor_controller.set_steering("right")
            motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
            sleep(1)
            
            # Turn straight
            motor_controller.set_steering("left")
            motor_controller.set_speed(default_speed_pwm-0.1) # Set speed back to default
            sleep(0.5)
    except KeyboardInterrupt:
        pass

    # Terminate motor controller and clean up GPIO settings
    motor_controller.stop()
    GPIO.cleanup()