import math
import cv2
import numpy as np
from camera import ThreadedCamera

class LaneTracker:
    def __init__(self):
        self.cam = ThreadedCamera(0)        # FIX 1: use ThreadedCamera, not cv2.VideoCapture
        self.smoothed_error = 0
        self.alpha = 0.3
        self.camera_offset_px = 5
        
        print("Warming up the dumb camera")
        for i in range(10):
          self.cam.get_frame()

        print("--- Visual Fix: Curved HSV Pipeline Active ---")

    def get_lane_data(self):
        frame = self.cam.get_frame()        # FIX 1: ThreadedCamera API
        if frame is None:
            return None, None, None

        # 1. Setup Frame and HSV
        h, w = frame.shape[:2]
        mid_x = (w/2)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Blue Masking and ROI
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        roi_mask = np.zeros_like(mask)
        cv2.rectangle(roi_mask, (0, int(h * 0.6)), (w, h), 255, -1)
        masked_blue = cv2.bitwise_and(mask, roi_mask)

        # 3. Initialize Overlay
        overlay = np.zeros_like(frame)
        center_fit = None

        # 4. Extract and Cluster Pixels
        pixel_y, pixel_x = np.where(masked_blue == 255)

        if len(pixel_x) > 50:
            sorted_idx = np.argsort(pixel_x)
            sx, sy = pixel_x[sorted_idx], pixel_y[sorted_idx]

            # Find gap between two lanes
            gaps = np.where(np.diff(sx) > 40)[0]
            plot_y = np.linspace(int(h * 0.4), h - 1, 20)

            try:
                if len(gaps) > 0:
                  # Dual Lane Detection
                  split = gaps[0] + 1
                  lx, ly = sx[:split], sy[:split]
                  rx, ry = sx[split:], sy[split:]
              
                  if len(ly) > 10 and len(ry) > 10:
                      l_fit = np.polyfit(ly, lx, 2)
                      r_fit = np.polyfit(ry, rx, 2)
              
                      # --- CENTERING CONSTRAINT ---
                      # At each y level, measure distance from center to each lane
                      # If unequal, shift the center fit until they match
                      check_y = np.linspace(int(h * 0.6), h - 1, 10)  # sample points in ROI
              
                      l_pts = np.polyval(l_fit, check_y)   # left lane x positions
                      r_pts = np.polyval(r_fit, check_y)   # right lane x positions
              
                      dist_to_left  = np.mean(np.polyval((l_fit + r_fit) / 2, check_y) - l_pts)
                      dist_to_right = np.mean(r_pts - np.polyval((l_fit + r_fit) / 2, check_y))
              
                      imbalance = dist_to_left - dist_to_right   # positive = too close to right
                      correction = imbalance / 2                  # shift center by half the gap
              
                      center_fit = (l_fit + r_fit) / 2
                      center_fit[2] += correction                 # shift the constant term only
              
#                      print(f"[Center] L:{dist_to_left:.1f} R:{dist_to_right:.1f} Imbalance:{imbalance:.1f} Correction:{correction:.1f}px")
              
                      # Visualize
                      l_path = np.array([np.transpose(np.vstack([np.polyval(l_fit, plot_y), plot_y]))], np.int32)
                      r_path = np.array([np.transpose(np.vstack([np.polyval(r_fit, plot_y), plot_y]))], np.int32)
                      cv2.polylines(overlay, l_path, False, (255, 0, 0), 2)
                      cv2.polylines(overlay, r_path, False, (255, 0, 0), 2)
            except Exception:
                pass

                
        if center_fit is not None:
          center_at_bottom = np.polyval(center_fit, h-1)
                    
        # 5. Draw the Center Trajectory (Green Curve)
        if center_fit is not None:
            c_pts = np.polyval(center_fit, plot_y)
            c_path = np.array([np.transpose(np.vstack([c_pts, plot_y]))], np.int32)
            cv2.polylines(overlay, c_path, False, (0, 255, 0), 5)

        # 6. Red Detection
        r_mask = cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 70, 50]),   np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255]))
        )
        overlay[r_mask > 0] = (0, 0, 255)

        final_view = cv2.addWeighted(frame, 0.7, overlay, 1.0, 0)

        # FIX 3: only update smoothed_error when we actually see a lane
        angle = self.calculate_steering_angle(center_fit, frame.shape)
        if angle is not None:
            self.smoothed_error = (self.alpha * (angle - 90) + (1 - self.alpha) * self.smoothed_error)

        steering_angle = 90 + self.smoothed_error
        return steering_angle, np.sum(r_mask) / 255, final_view

    def calculate_steering_angle(self, center_fit, frame_shape):
        """
        Calculates the steering angle from the polynomial center path.
        90 degrees = STRAIGHT. <90 = LEFT. >90 = RIGHT.
        """
        h, w = frame_shape[:2]
        mid_x = w / 2

        if center_fit is None:
            return None   

        target_y = int(h * 0.6)
        target_x = np.polyval(center_fit, target_y)

        dx = target_x - mid_x
        dy = h - target_y

        angle_rad = math.atan2(dx, dy)
        angle_deg = math.degrees(angle_rad)

        steering_angle = int(angle_deg + 90)

        return max(50, min(130, steering_angle))

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
