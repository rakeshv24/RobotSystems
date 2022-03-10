import cv2
import numpy as np


class ImageTracer:
    def __init__(self):
        self.gray_image = None
        self.blur_image = None
        self.resized_image = None
        self.org_image = None
        self.thresh_image = None
        self.contours = None
        self.width = 640
        self.height = 480

    def detect_shape(self, c):
        shape = "unidentified"
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)

        if len(approx) == 3:
            shape = "triangle"
        elif len(approx) == 4:
            (x, y, w, h) = cv2.boundingRect(approx)
            ar = w / float(h)
            shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
        elif len(approx) == 5:
            shape = "pentagon"
        else:
            shape = "circle"
        
        return shape
    
    def find_contours(self):
        self.contours = cv2.findContours(self.thresh_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        for cnt in self.contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            if len(approx) == 5:
                print("pentagon")
                cv2.drawContours(self.org_image, [cnt], 0, 255, -1)
            elif len(approx) == 3:
                print("triangle")
                cv2.drawContours(self.org_image, [cnt], 0, (0, 255, 0), -1)
            elif len(approx) == 4:
                print("square")
                cv2.drawContours(self.org_image, [cnt], 0, (0 , 0, 255), -1)
            elif len(approx) == 9:
                print("half-circle")
                cv2.drawContours(self.org_image, [cnt], 0, (255, 255, 0), -1)
            elif len(approx) > 15:
                print("circle")
                cv2.drawContours(self.org_image, [cnt], 0, (0, 255, 255), -1)

    def grab_image(self):
        self.cap = cv2.VideoCapture(0)
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.process_image(frame)
                self.feature_detection()
                self.show_image()                
                if cv2.waitKey(27) & 0xFF == ord('q'):
                    break
            else:
                break
        self.close_stream()
            
    def close_stream(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def process_image(self, frame):
        self.resized_image = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
        self.org_image = self.resized_image.copy()
        self.gray_image = cv2.cvtColor(self.resized_image, cv2.COLOR_BGR2GRAY)
        self.blur_image = cv2.GaussianBlur(self.gray_image, (5, 5), 0)
        self.thresh_image = cv2.threshold(self.blur_image, 60, 255, cv2.THRESH_BINARY)[1]
    
    def feature_detection(self):
        gray = np.float32(self.gray_image)
        corners = cv2.goodFeaturesToTrack(gray, 10, 0.01, 10)
        corners = np.int0(corners).reshape(-1, 2)
        corners = corners[np.argsort(corners[:, 0])]
        
        for i in corners:
            x, y = i.ravel()
            cv2.circle(self.org_image, (x, y), 4, 200, -1)
        return corners
    
    def get_waypoints(self):
        pass
    
    def show_image(self):
        cv2.imshow('frame', self.org_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    it = ImageTracer()
    it.grab_image()