import time
import cv2
from picamera2 import Picamera2, Preview
import numpy as np
from collections import deque


class Camera:
    # Default resolution
    def __init__(self, resolution=(1640, 1232), format="RGB888"):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": resolution, "format": format},
            controls={"FrameDurationLimits": (8333, 8333)}  # ~120fps
        )
        self.picam2.configure(config)


        # Orange HSV range (adjust if needed)
        self.lower_orange = np.array([5, 150, 150])   
        self.upper_orange = np.array([25, 255, 255])  


        # History of positions (for smoothing)
        self.queue = deque(maxlen=16)
        self.queue.append((100, 75))  


        self.picam2.start()    


    def take_picture(self):
        image = self.picam2.capture_array()
        scale = 200.0 / image.shape[1]
        frame_resized = cv2.resize(image, (200, int(image.shape[0] * scale)))
        return frame_resized


    def display(self, image, window_name="Camera Output"):
        cv2.imshow(window_name, image)
        cv2.waitKey(1) 


    def display_draw(self, mask, center, window_name="Tracked Mask Output"):
        """Draw crosshair on the mask instead of RGB frame"""
        x, y = center
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # convert mask to BGR for drawing
        cv2.line(mask_bgr, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(mask_bgr, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
        cv2.imshow(window_name, mask_bgr)
        cv2.waitKey(1) 


    def terminate(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()


    def coordinate(self, image):
        # Apply Gaussian blur
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Convert from BGR to HSV
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)


        # Mask for orange → white, everything else → black
        mask_hsv = cv2.inRange(frame_hsv, self.lower_orange, self.upper_orange)


        # Morphological filtering (reduce noise)
        mask_clean = cv2.erode(mask_hsv, None, iterations=1)
        mask_clean = cv2.dilate(mask_clean, None, iterations=1)


        # Find contours on the mask
        valid_detections = []
        contours, _ = cv2.findContours(mask_clean.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)


            # Ignore blobs too small or too big
            if radius < 5 or radius > 100:
                continue


            # Compute circularity
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:
                continue


            # Center of bounding box
            x, y, w, h = cv2.boundingRect(contour)
            valid_detections.append((area, (int(x + w / 2), int(y + h / 2))))


        # Pick the largest detection
        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]
            self.queue.append(best_center)
        else:
            # No detection → repeat last position
            self.queue.append(self.queue[-1])


        return self.queue[-1], mask_clean
        


if __name__ == "__main__":import time
import cv2
from picamera2 import Picamera2, Preview
import numpy as np
from collections import deque


class Camera:
    # Default resolution
    def __init__(self, resolution=(1640, 1232), format="RGB888"):
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": resolution, "format": format},
            controls={"FrameDurationLimits": (8333, 8333)}  # ~120fps
        )
        self.picam2.configure(config)


        # Orange HSV range (adjust if needed)
        self.lower_orange = np.array([5, 150, 150])   
        self.upper_orange = np.array([25, 255, 255])  


        # History of positions (for smoothing)
        self.queue = deque(maxlen=16)
        self.queue.append((100, 75))  


        self.picam2.start()    


    def take_picture(self):
        image = self.picam2.capture_array()
        scale = 200.0 / image.shape[1]
        frame_resized = cv2.resize(image, (200, int(image.shape[0] * scale)))
        return frame_resized


    def display(self, image, window_name="Camera Output"):
        cv2.imshow(window_name, image)
        cv2.waitKey(1) 


    def display_draw(self, mask, center, window_name="Tracked Mask Output"):
        """Draw crosshair on the mask instead of RGB frame"""
        x, y = center
        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # convert mask to BGR for drawing
        cv2.line(mask_bgr, (x - 10, y), (x + 10, y), (0, 0, 255), 2)
        cv2.line(mask_bgr, (x, y - 10), (x, y + 10), (0, 0, 255), 2)
        cv2.imshow(window_name, mask_bgr)
        cv2.waitKey(1) 


    def terminate(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()


    def coordinate(self, image):
        # Apply Gaussian blur
        frame_blurred = cv2.GaussianBlur(image, (3, 3), 0)
        
        # Convert from BGR to HSV
        frame_hsv = cv2.cvtColor(frame_blurred, cv2.COLOR_BGR2HSV)


        # Mask for orange → white, everything else → black
        mask_hsv = cv2.inRange(frame_hsv, self.lower_orange, self.upper_orange)


        # Morphological filtering (reduce noise)
        mask_clean = cv2.erode(mask_hsv, None, iterations=1)
        mask_clean = cv2.dilate(mask_clean, None, iterations=1)


        # Find contours on the mask
        valid_detections = []
        contours, _ = cv2.findContours(mask_clean.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            (x, y), radius = cv2.minEnclosingCircle(contour)
            radius = int(radius)


            # Ignore blobs too small or too big
            if radius < 5 or radius > 100:
                continue


            # Compute circularity
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter == 0:
                continue
            circularity = (4 * np.pi * area) / (perimeter ** 2)
            if circularity < 0.6:
                continue


            # Center of bounding box
            x, y, w, h = cv2.boundingRect(contour)
            valid_detections.append((area, (int(x + w / 2), int(y + h / 2))))


        # Pick the largest detection
        if valid_detections:
            best_center = max(valid_detections, key=lambda item: item[0])[1]
            self.queue.append(best_center)
        else:
            # No detection → repeat last position
            self.queue.append(self.queue[-1])


        # Show tracking on mask
        self.display_draw(mask_clean, self.queue[-1])


        # Return only (x, y) so main.py works without change
        return self.queue[-1]
        


if __name__ == "__main__":

    
    cam = Camera()


    try:
        while True:
            img = cam.take_picture()
            c = cam.coordinate(img)   # now returns only (x, y)
            print(c)
            
            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.terminate()


    cam = Camera()


    try:
        while True:
            img = cam.take_picture()
            c, mask = cam.coordinate(img)
            cam.display_draw(mask, c)   # <-- now shows crosshair on binary mask
            print(c)
            
            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.terminate()