import math
import cv2
import numpy as np

class LaneTracker:
    def __init__(self):
        # Recommended resolution for Pi 5 [cite: 15, 170]
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # EMA smoothing for steering angle
        self.smoothed_error = 0
        self.alpha = 0.2   # smaller = smoother, slower response

        print("--- HSV Blue Lane + Red Stop Pipeline Active ---")

    def get_lane_data(self):
        ret, frame = self.cam.read()
        if not ret:
            return None, None, None

        h, w = frame.shape[:2]
        mid_x = w / 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 1. Blue Masking & ROI
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        roi_mask = np.zeros_like(blue_mask)
        cv2.rectangle(roi_mask, (0, int(h*0.6)), (w, h), 255, -1)
        masked_blue = cv2.bitwise_and(blue_mask, roi_mask)

        # 2. Hough Line Detection 
        segments = cv2.HoughLinesP(masked_blue, 1, np.pi/180, 20, minLineLength=20, maxLineGap=50)

        left_pts_x, left_pts_y = [], []
        right_pts_x, right_pts_y = [], []

        if segments is not None:
            for seg in segments:
                for x1, y1, x2, y2 in seg:
                    slope = (y2 - y1) / (x2 - x1) if x2 != x1 else 999
                    if abs(slope) < 0.3: continue 

                    if x1 < mid_x and x2 < mid_x:
                        left_pts_x.extend([x1, x2])
                        left_pts_y.extend([y1, y2])
                    elif x1 >= mid_x and x2 >= mid_x:
                        right_pts_x.extend([x1, x2])
                        right_pts_y.extend([y1, y2])

        # 3. Polynomial Fitting to Hough Points
        overlay = np.zeros_like(frame)
        plot_y = np.linspace(int(h*0.6), h - 1, 20)
        
        # FIX 1: Initialize center_fit to None so it exists even if detection fails
        center_fit = None 

        try:
            if len(left_pts_y) > 4 and len(right_pts_y) > 4:
                left_fit = np.polyfit(left_pts_y, left_pts_x, 2)
                right_fit = np.polyfit(right_pts_y, right_pts_x, 2)
                center_fit = (left_fit + right_fit) / 2

                l_path = np.array([np.transpose(np.vstack([np.polyval(left_fit, plot_y), plot_y]))], np.int32)
                r_path = np.array([np.transpose(np.vstack([np.polyval(right_fit, plot_y), plot_y]))], np.int32)
                c_path = np.array([np.transpose(np.vstack([np.polyval(center_fit, plot_y), plot_y]))], np.int32)

                cv2.polylines(overlay, l_path, False, (100, 100, 255), 4) 
                cv2.polylines(overlay, r_path, False, (100, 100, 255), 4)
                cv2.polylines(overlay, c_path, False, (0, 255, 0), 5)     
        except Exception as e:
            pass

        # 4. Red Detection 
        r_mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        r_mask2 = cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(r_mask1, r_mask2)
        overlay[red_mask > 0] = (0, 0, 255)

        # 5. EMA Smoothing & Final View
        # Get the angle using the safe function
        raw_angle = self.calculate_steering_angle(center_fit, frame.shape)
        
        # Initialize EMA tracking if it doesn't exist
        if not hasattr(self, 'smoothed_angle'):
            self.smoothed_angle = 90.0

        # FIX 2: Smooth the ANGLE, not the raw pixel error
        self.smoothed_angle = (self.alpha * raw_angle + (1 - self.alpha) * self.smoothed_angle)
        
        final_view = cv2.addWeighted(frame, 0.7, overlay, 1.0, 0)
        cv2.line(final_view, (int(mid_x), 0), (int(mid_x), h), (255, 255, 255), 1)

        # Return the Angle to the PID controller!
        return self.smoothed_angle, (np.sum(red_mask)/255), final_view

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

    def clean_up(self):
        self.cam.release()
        cv2.destroyAllWindows()

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
