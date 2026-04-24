import cv2
import numpy as np

def test():
	cam = cv2.VideoCapture(0)
	cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
	cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
	
	print("Press 'q' to quit, if you dare")

	while True:
		ret, frame = cam.read()
		if not ret: break

		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blur = cv2.GaussianBlur(gray, (5,5), 0)
		edges = cv2.Canny(blur, 50, 150)
		
		height, width = edges.shape
		mask = np.zeros_like(edges)
		polygon = np.array([[(0, height), (width, height), (width, height//2), (0, height//2)]], np.int32)
		cv2.fillPoly(mask, polygon, 255)
		cropped_edges = cv2.bitwise_and(edges, mask)
		line_segments = cv2.HoughLinesP(cropped_edges, 1, np.pi/180, 15,  np.array([]), minLineLength = 10, maxLineGap = 40)
		
		lane_lines = []
		if line_segments is not None:
			for line in line_segments:
				for x1, y1, x2, y2 in line:
					if x1 == x2: continue
					slope = (y2-y1)/(x2-x1)
					if abs(slope) > 0.1:
						lane_lines.append(line)
		
		line_img = np.zeros_like(frame)
		if line_segments is not None:
			for line in line_segments:
				for x1, y1, x2, y2 in line:
					cv2.line(line_img, (x1,y1), (x2,y2), (0, 255, 0), 2)
		
		output = cv2.addWeighted(frame, 0.8, line_img, 1, 1)
		cv2.imshow("Cool View", output)

		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	
	cam.release()
	cv2.destroyAllWindows()

if __name__ == "__main__":
	test()
