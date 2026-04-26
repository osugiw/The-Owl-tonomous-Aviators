import math
import cv2
import numpy as np

class LaneTracker:
    def __init__(self):
        # Recommended resolution for Pi 5 [cite: 15, 170]
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.smoothed_error = 0
        self.alpha = 0.2  

        print("--- Visual Fix: Curved HSV Pipeline Active ---")

    def get_lane_data(self):
        ret, frame = self.cam.read()
        if not ret:
            return None, None, None

        # 1. HSV Conversion
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Blue Masking
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. ROI (Bottom 40%)
        height, width = frame.shape[:2]
        roi_mask = np.zeros_like(blue_mask)
        cv2.rectangle(roi_mask, (0, int(height*0.6)), (width, height), 255, -1)
        masked_blue = cv2.bitwise_and(blue_mask, roi_mask)

        # 4. Polynomial Curve Fitting
        pixel_y, pixel_x = np.where(masked_blue == 255)
        mid_x = width / 2
        
        overlay = np.zeros_like(frame)
        plot_y = np.linspace(int(height*0.6), height - 1, 20)
        
        # CRITICAL FIX: Use None instead of 0 to prevent the 0-d array crash
        center_fit = None 

        # Separate Left and Right
        left_x, left_y = pixel_x[pixel_x < mid_x], pixel_y[pixel_x < mid_x]
        right_x, right_y = pixel_x[pixel_x >= mid_x], pixel_y[pixel_x >= mid_x]

        if len(left_y) > 50 and len(right_y) > 50:
            # Calculate the 2nd degree polynomial fits
            left_fit = np.polyfit(left_y, left_x, 2)
            right_fit = np.polyfit(right_y, right_x, 2)
            center_fit = (left_fit + right_fit) / 2

            # --- VISUAL FIX: Draw Individual Blue Lanes ---
            l_pts_x = np.polyval(left_fit, plot_y)
            r_pts_x = np.polyval(right_fit, plot_y)
            
            l_curve = np.array([np.transpose(np.vstack([l_pts_x, plot_y]))], np.int32)
            r_curve = np.array([np.transpose(np.vstack([r_pts_x, plot_y]))], np.int32)
            
            # Draw left and right lines in BLUE (OpenCV uses BGR: 255, 0, 0)
            cv2.polylines(overlay, l_curve, False, (255, 0, 0), 3)
            cv2.polylines(overlay, r_curve, False, (255, 0, 0), 3)
            
            # Draw the curved center path in GREEN (0, 255, 0)
            c_pts_x = np.polyval(center_fit, plot_y)
            center_curve = np.array([np.transpose(np.vstack([c_pts_x, plot_y]))], np.int32)
            cv2.polylines(overlay, center_curve, False, (0, 255, 0), 5)

        # 5. Red Detection
        r_mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        r_mask2 = cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(r_mask1, r_mask2)
        red_area = np.sum(red_mask) / 255
        overlay[red_mask > 0] = (0, 0, 255) # Red
        
        # 6. Calculate Steering Angle & Apply EMA Smoothing
        raw_angle = self.calculate_steering_angle(center_fit, frame.shape)
        
        # Initialize the smoothing variable if this is the first loop
        if not hasattr(self, 'smoothed_angle'):
            self.smoothed_angle = 90.0
            
        self.smoothed_angle = (self.alpha * raw_angle + (1 - self.alpha) * self.smoothed_angle) 

        # 7. Weighted Synthesis & Text Display
        final_view = cv2.addWeighted(frame, 0.7, overlay, 1.0, 0)
        cv2.line(final_view, (int(mid_x), 0), (int(mid_x), height), (255, 255, 255), 1)
        
        # Display the calculated angle in White so it's readable
        cv2.putText(final_view, f"Steering angle: {self.smoothed_angle:.1f} deg", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return self.smoothed_angle, red_area, final_view

    def clean_up(self):
        self.cam.release()
        cv2.destroyAllWindows()

    def calculate_steering_angle(self, center_fit, frame_shape):
        """
        Calculates the steering angle from the polynomial center path.
        90 degrees is STRAIGHT. <90 is LEFT. >90 is RIGHT.
        """
        h, w = frame_shape[:2]
        mid_x = w / 2

        if center_fit is None:
            return 90  # Default to straight if no path is found

        # 1. Select a 'Look-Ahead' point (Top of the ROI)
        # Looking at the horizon (h*0.6) makes steering smoother
        target_y = int(h * 0.6)
        target_x = np.polyval(center_fit, target_y)

        # 2. Geometric Math
        # dx: horizontal distance from center
        # dy: vertical distance from the car's 'camera' position (bottom of frame)
        dx = target_x - mid_x
        dy = h - target_y

        # 3. Calculate Angle using inverse tangent
        # math.atan2(dx, dy) gives the angle in radians relative to vertical center
        angle_rad = math.atan2(dx, dy)
        angle_deg = math.degrees(angle_rad)

        # 4. Normalize to 90-degree center
        steering_angle = int(angle_deg + 90)

        # 5. Safety Clamp
        # Prevents the servo from trying to turn past its physical limits
        return max(45, min(135, steering_angle))

if __name__ == "__main__":
    tracker = LaneTracker()
    try:
        while True:
            error, red, img = tracker.get_lane_data()
            if img is not None:
                print(f"Error: {error:5.1f} | Red Area: {red:6.0f}", end="\r")
                cv2.imshow("Perfected View", img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    finally:
        tracker.clean_up()
