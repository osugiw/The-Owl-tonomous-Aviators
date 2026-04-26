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

        # 1. Setup Frame and HSV
        h, w = frame.shape[:2]
        mid_x = w / 2
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Blue Masking and ROI (Expanded to 40% for better look-ahead)
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        
        roi_mask = np.zeros_like(mask)
        cv2.rectangle(roi_mask, (0, int(h*0.4)), (w, h), 255, -1)
        masked_blue = cv2.bitwise_and(mask, roi_mask)

        # 3. Initialize Overlay (Crucial to avoid NameError)
        overlay = np.zeros_like(frame)
        center_fit = None
        
        # 4. Extract and Cluster Pixels
        pixel_y, pixel_x = np.where(masked_blue == 255)
        
        if len(pixel_x) > 50:
            sorted_idx = np.argsort(pixel_x)
            sx, sy = pixel_x[sorted_idx], pixel_y[sorted_idx]
            
            # Find gap between two lanes
            gaps = np.where(np.diff(sx) > 40)[0]
            plot_y = np.linspace(int(h*0.4), h - 1, 20)

            try:
                if len(gaps) > 0:
                    # Dual Lane Detection
                    split = gaps[0] + 1
                    lx, ly = sx[:split], sy[:split]
                    rx, ry = sx[split:], sy[split:]

                    if len(ly) > 10 and len(ry) > 10:
                        l_fit = np.polyfit(ly, lx, 2)
                        r_fit = np.polyfit(ry, rx, 2)
                        center_fit = (l_fit + r_fit) / 2

                        # --- Visualizing the 2 Lane Strips ---
                        l_path = np.array([np.transpose(np.vstack([np.polyval(l_fit, plot_y), plot_y]))], np.int32)
                        r_path = np.array([np.transpose(np.vstack([np.polyval(r_fit, plot_y), plot_y]))], np.int32)
                        cv2.polylines(overlay, l_path, False, (255, 0, 0), 2) # Blue detected edge
                        cv2.polylines(overlay, r_path, False, (255, 0, 0), 2) # Blue detected edge
                
                else:
                    # Single Lane Detection + Virtual Offset
                    s_fit = np.polyfit(sy, sx, 2)
                    center_fit = s_fit.copy()
                    offset = 120 if np.mean(sx) < mid_x else -120
                    center_fit[2] += offset
                    
                    # Visualize the single detected strip
                    s_path = np.array([np.transpose(np.vstack([np.polyval(s_fit, plot_y), plot_y]))], np.int32)
                    cv2.polylines(overlay, s_path, False, (255, 100, 0), 2)

            except Exception: pass

        # 5. Draw the Center Trajectory (Green Curve)
        if center_fit is not None:
            c_pts = np.polyval(center_fit, plot_y)
            c_path = np.array([np.transpose(np.vstack([c_pts, plot_y]))], np.int32)
            cv2.polylines(overlay, c_path, False, (0, 255, 0), 5)

        # 6. Red Detection and Synthesis
        r_mask = cv2.bitwise_or(cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255])),
                                cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255])))
        overlay[r_mask > 0] = (0, 0, 255)
        
        final_view = cv2.addWeighted(frame, 0.7, overlay, 1.0, 0)
        
        # FIX 1: Turn Memory Logic
        angle = self.calculate_steering_angle(center_fit, frame.shape)
        
        # Only update the error if we actually see a line. 
        # If angle is None, smoothed_error stays exactly where it was (holding the turn)
        if angle is not None:
            self.smoothed_error = (self.alpha * (angle - 90) + (1 - self.alpha) * self.smoothed_error)
        
        return self.smoothed_error + 90, np.sum(r_mask)/255, final_view

    def calculate_steering_angle(self, center_fit, frame_shape):
        """
        Calculates the steering angle from the polynomial center path.
        90 degrees is STRAIGHT. <90 is LEFT. >90 is RIGHT.
        """
        h, w = frame_shape[:2]
        mid_x = w / 2

        if center_fit is None:
            return None  # Default to straight if no path is found

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
