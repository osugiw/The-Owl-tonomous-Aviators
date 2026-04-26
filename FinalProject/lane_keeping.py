import cv2
import numpy as np

class LaneTracker:
    def __init__(self):
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("--- Perfected Blue Lane + Red Stop Pipeline Module Active ---")

    def get_lane_data(self):
        ret, frame = self.cam.read()
        if not ret:
            return None, None, None

        #Processing (Slides 16, 19, 27)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 60, 120) 
        
        #ROI, shifts focus to bottom 40% (Slide 30)
        height, width = edges.shape
        mask = np.zeros_like(edges)
        polygon = np.array([[(0, height), (width, height), (width, int(height*0.6)), (0, int(height*0.6))]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)

        #Them good hough lines (Slide 31)
        line_segments = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 20, np.array([]), minLineLength=15, maxLineGap=50)
        
        line_img = np.zeros_like(frame)
        left_fit = []
        right_fit = []

        if line_segments is not None:
            for line in line_segments:
                for x1, y1, x2, y2 in line:
                    if x1 == x2: continue
                    slope = (y2 - y1) / (x2 - x1)
                    
                    if -3.0 < slope < -0.5: 
                        left_fit.append((slope, y1 - slope * x1)) 
                        cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 3)
                    elif 0.5 < slope < 3.0: 
                        right_fit.append((slope, y1 - slope * x1))
                        cv2.line(line_img, (x1, y1), (x2, y2), (255, 0, 0), 3)

        #Calculates lane center (Slide 40)
        lane_center = width / 2
        if len(left_fit) > 0 and len(right_fit) > 0:
            left_avg = np.average(left_fit, axis=0)
            right_avg = np.average(right_fit, axis=0)
            left_x = (height - left_avg[1]) / left_avg[0]
            right_x = (height - right_avg[1]) / right_avg[0]
            lane_center = (left_x + right_x) / 2
        elif len(left_fit) > 0:
            left_avg = np.average(left_fit, axis=0)
            left_x = (height - left_avg[1]) / left_avg[0]
            lane_center = left_x + 100 
        elif len(right_fit) > 0:
            right_avg = np.average(right_fit, axis=0)
            right_x = (height - right_avg[1]) / right_avg[0]
            lane_center = right_x - 100

        #Red mask pour le detection of red box
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_r1 = cv2.inRange(hsv, np.array([0, 70, 50]), np.array([10, 255, 255])) 
        mask_r2 = cv2.inRange(hsv, np.array([155, 70, 50]), np.array([180, 255, 255])) 
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        
        red_overlay = np.zeros_like(frame)
        red_overlay[red_mask > 0] = (0, 0, 255)

        #Final view construction
        canny_bgr = cv2.cvtColor(cropped_edges, cv2.COLOR_GRAY2BGR)
        final_view = cv2.addWeighted(canny_bgr, 0.4, line_img, 1, 0)
        final_view = cv2.addWeighted(final_view, 1, red_overlay, 0.6, 0)

        cv2.line(final_view, (int(width/2), height), (int(width/2), 0), (255, 255, 255), 1)
        cv2.circle(final_view, (int(lane_center), int(height * 0.8)), 5, (0, 255, 255), -1)

        steering_error = lane_center - (width / 2)
        red_area = np.sum(red_mask) / 255

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