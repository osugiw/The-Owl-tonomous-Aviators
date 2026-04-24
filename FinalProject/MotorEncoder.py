import RPi.GPIO as GPIO
import time

class MotorEncoder():
    def __init__(self):
        self.param_path = "/sys/module/EncoderDriver/parameters/enc_count"
        self.last_count = self._read_enc()
        self.last_time = time.time()
        self.holes_per_rev = 20

    def _read_enc(self):
        """Read the raw count from the kernel module"""
        try:
            with open(self.param_path, "r") as f:
                return int(f.read().strip())
        except FileNotFoundError:
            print(f"Error: {self.param_path} not found")
            return 0

    def calculate_rpm(self):
        """Calculate the RPM"""
        current_count = self._read_enc()
        current_time = time.time()

        # Calculate deltas based on previous count
        delta_count = current_count - self.last_count
        delta_time = current_time - self.last_time

        if delta_time <= 0:
            return 0.0

        # RPM calculation
        rpm = (delta_count / delta_time) * (60 / self.holes_per_rev)

        # Update time and count
        self.last_count = current_count
        self.last_time = current_time

        return rpm

# Main loop
if __name__ == "__main__":
    reader = MotorEncoder()
    try:
        while True:
            time.sleep(1) # Measure every second
            print(f"Current Speed: {reader.calculate_rpm():.2f} RPM")
    except KeyboardInterrupt:
        print("\nStopped.")
