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

        # 1. HSV Conversion [cite: 219]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Blue Masking
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # 3. ROI (Bottom 40%) [cite: 30]
        height, width = frame.shape[:2]
        roi_mask = np.zeros_like(blue_mask)
        cv2.rectangle(roi_mask, (0, int(height*0.6)), (width, height), 255, -1)
        masked_blue = cv2.bitwise_and(blue_mask, roi_mask)

        # 4. Polynomial Curve Fitting [cite: 185]
        pixel_y, pixel_x = np.where(masked_blue == 255)
        mid_x = width / 2
        
        # Create a clean visualization layer
        overlay = np.zeros_like(frame)
        plot_y = np.linspace(int(height*0.6), height - 1, 20)
        raw_error = 0

        try:
            # Separate Left and Right
            left_x, left_y = pixel_x[pixel_x < mid_x], pixel_y[pixel_x < mid_x]
            right_x, right_y = pixel_x[pixel_x >= mid_x], pixel_y[pixel_x >= mid_x]

            if len(left_y) > 50 and len(right_y) > 50:
                left_fit = np.polyfit(left_y, left_x, 2)
                right_fit = np.polyfit(right_y, right_x, 2)
                center_fit = (left_fit + right_fit) / 2
                pts = np.polyval(center_fit, plot_y)
                
                # Draw the curved path
                curve_pts = np.array([np.transpose(np.vstack([pts, plot_y]))], np.int32)
                cv2.polylines(overlay, curve_pts, False, (0, 255, 0), 5)
                raw_error = pts[-1] - mid_x
        except:
            pass

        # 5. Red Detection [cite: 35, 218]
        r_mask1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255]))
        r_mask2 = cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255]))
        red_mask = cv2.bitwise_or(r_mask1, r_mask2)
        red_area = np.sum(red_mask) / 255
        
        # Apply red color to the overlay
        overlay[red_mask > 0] = (0, 0, 255)

        # 6. Smooth the Error
        self.smoothed_error = (self.alpha * raw_error + (1 - self.alpha) * self.smoothed_error)

        # 7. Weighted Synthesis (Combines everything while keeping original colors)
        # 0.7 original frame + 1.0 overlay (lanes/red)
        final_view = cv2.addWeighted(frame, 0.7, overlay, 1.0, 0)
        
        # Center reference
        cv2.line(final_view, (int(mid_x), 0), (int(mid_x), height), (255, 255, 255), 1)

        return self.smoothed_error, red_area, final_view

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
