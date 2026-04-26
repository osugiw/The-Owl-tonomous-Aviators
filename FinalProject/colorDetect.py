import math
import cv2
import numpy as np

class LaneTracker:
    def __init__(self):
        # Initializing at the recommended resolution for Pi 5 [cite: 15, 170]
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

        # 1. Convert to HSV (Better for identifying specific track colors) [cite: 219]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Blue Color Detection (Reference Code Logic)
        lower_blue = np.array([90, 50, 50], dtype="uint8") 
        upper_blue = np.array([130, 255, 255], dtype="uint8") 
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue) 

        # 3. Edge Detection on the Color Mask
        edges = cv2.Canny(blue_mask, 50, 100) 
        
        # 4. ROI: focus on bottom 40% of frame [cite: 30]
        height, width = edges.shape
        roi_mask = np.zeros_like(edges)
        polygon = np.array([[(0, height), (width, height), 
                             (width, int(height*0.6)), (0, int(height*0.6))]], np.int32)
        cv2.fillPoly(roi_mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, roi_mask)

        # 5. Hough Line Segments [cite: 146, 181]
        line_segments = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 20, 
                                        minLineLength=15, maxLineGap=50)
        
        line_img = np.zeros_like(frame)
        left_fit = []
        right_fit = []

        if line_segments is not None:
            for line in line_segments:
                for x1, y1, x2, y2 in line:
                    if x1 == x2: continue
                    slope = (y2 - y1) / (x2 - x1)
                    
                    # Filtering based on expected lane angles
                    if -5.0 < slope < -0.1: 
                        left_fit.append((slope, y1 - slope * x1)) 
                        cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    elif 0.1 < slope < 5.0: 
                        right_fit.append((slope, y1 - slope * x1))
                        cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 3)

        # 6. Lane Center Calculation (Slide 40 Logic)
        mid_x = width / 2
        lane_center = mid_x # Default to center
        
        if len(left_fit) > 0 and len(right_fit) > 0:
            left_avg = np.average(left_fit, axis=0)
            right_avg = np.average(right_fit, axis=0)
            # Find x positions at the bottom of the ROI
            y_eval = height
            left_x = (y_eval - left_avg[1]) / left_avg[0]
            right_x = (y_eval - right_avg[1]) / right_avg[0]
            lane_center = (left_x + right_x) / 2
        elif len(left_fit) > 0:
            left_avg = np.average(left_fit, axis=0)
            left_x = (height - left_avg[1]) / left_avg[0]
            lane_center = left_x + 120 # Offset from left line
        elif len(right_fit) > 0:
            right_avg = np.average(right_fit, axis=0)
            right_x = (height - right_avg[1]) / right_avg[0]
            lane_center = right_x - 120 # Offset from right line

        # 7. Red Detection for Stop Boxes [cite: 35, 218]
        mask_r1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255])) 
        mask_r2 = cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255])) 
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        red_area = np.sum(red_mask) / 255
        
        red_overlay = np.zeros_like(frame)
        red_overlay[red_mask > 0] = (0, 0, 255)

        # 8. Visual Synthesis
        final_view = cv2.addWeighted(frame, 0.4, line_img, 1, 0)
        final_view = cv2.addWeighted(final_view, 1, red_overlay, 0.6, 0)

        # 9. EMA Smoothing for Steering Error 
        raw_error = lane_center - mid_x
        self.smoothed_error = (self.alpha * raw_error + (1 - self.alpha) * self.smoothed_error)
        steering_error = self.smoothed_error

        # =========================
        # Curved Steering Path (Bezier Curve)
        # =========================
        car_center_x = int(mid_x)
        car_center_y = int(height)

        # curve_strength dictates how far the curve "pulls" towards the lane center
        curve_strength = steering_error * 0.8  
        path_length = 180

        # P0: Start (bottom center)
        p0 = (car_center_x, car_center_y)
        # P1: Control Point (middle height, offset by error)
        p1 = (int(car_center_x + curve_strength), int(car_center_y - path_length // 2))
        # P2: End Point (top of path, further offset)
        p2 = (int(car_center_x + curve_strength * 1.5), int(car_center_y - path_length))

        # Draw Bezier curve by connecting small line segments
        prev_point = p0
        for t in np.linspace(0, 1, 30):
            # Bezier formula: (1-t)^2*P0 + 2(1-t)t*P1 + t^2*P2
            x = int((1 - t)**2 * p0[0] + 2 * (1 - t) * t * p1[0] + t**2 * p2[0])
            y = int((1 - t)**2 * p0[1] + 2 * (1 - t) * t * p1[1] + t**2 * p2[1])
            current_point = (x, y)
            cv2.line(final_view, prev_point, current_point, (0, 255, 0), 4)
            prev_point = current_point

        # Yellow endpoint marker and center reference
        cv2.circle(final_view, p2, 8, (0, 255, 255), -1)
        cv2.line(final_view, (int(mid_x), 0), (int(mid_x), height), (255, 255, 255), 1)

        return steering_error, red_area, final_view

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
