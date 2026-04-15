import RPi.GPIO as GPIO
from time import sleep

# Minimum DC PWM
default_speed_pwm = 8.1
steering_forward = 7.5 
steering_right = 9.0
steering_left = 6.0 


# Use BCM pin numbering
GPIO.setmode(GPIO.BCM)

# Pin assignments
motor_speed_pin = 18
motor_steering_pin = 19

# Setup Motor Speed pin
GPIO.setup(motor_speed_pin, GPIO.OUT)
speed_pwm = GPIO.PWM(motor_speed_pin, 50) # set the frequency to 50 Hz 
speed_pwm.start(default_speed_pwm) # Start with 7.5% duty cycle

# Setup Motor Steering pin
GPIO.setup(motor_steering_pin, GPIO.OUT)
steering_pwm = GPIO.PWM(motor_steering_pin, 50) # set the frequency to 50 Hz 
steering_pwm.start(7.5) # Start with 7.5% duty cycle

try:
    while True:
        speed_pwm.ChangeDutyCycle(default_speed_pwm) # Change to 10% dc
        
        # Move forward
        steering_pwm.ChangeDutyCycle(steering_forward)
        sleep(0.5)
        
        # Stop
        speed_pwm.ChangeDutyCycle(0)
        sleep(0.5)
        
        # Turn left
        speed_pwm.ChangeDutyCycle(default_speed_pwm)
        steering_pwm.ChangeDutyCycle(steering_left)
        sleep(0.5)
        
        # Turn right
        steering_pwm.ChangeDutyCycle(steering_right)
        sleep(0.5)
        
        # Turn straight
        steering_pwm.ChangeDutyCycle(steering_left)
        sleep(0.5)
except KeyboardInterrupt:
    pass

speed_pwm.stop()
steering_pwm.stop()
GPIO.cleanup()
