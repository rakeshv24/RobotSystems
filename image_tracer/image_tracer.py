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
        self.width = 1280
        self.height = 720

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
        self.contours = cv2.findContours(self.canny_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        self.cont_imgs = []
        # self.hull_img = np.zeros((self.height, self.width), dtype=np.uint8)

        for i, c in enumerate(self.contours):
            epsilon = 0.1*cv2.arcLength(c,True)
            approx = cv2.approxPolyDP(c, epsilon, True)
            boundRect = cv2.boundingRect(approx)

            contArea = cv2.contourArea(c)
            minArea = 999

            # Filter blobs by area:
            if contArea > minArea:
                # Get the convex hull for the target contour:
                # hull = cv2.convexHull(c)
                # (Optional) Draw the hull:
                # color = (0, 0, 255)
                # cv2.polylines(self.org_image, [hull], True, color, 2)
                
                # Draw the points:
                # cv2.drawContours(self.hull_img, [hull], 0, 255, 2)
                
                cont_img = np.zeros((self.height, self.width), dtype=np.uint8)
                cv2.drawContours(cont_img, [c], 0, 255, 0)
                
                self.cont_imgs.append(cont_img)
                
    def grab_image(self):
        self.cap = cv2.VideoCapture(2)
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                self.process_image(frame)
                self.find_contours()
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
        self.thresh_image = cv2.threshold(self.blur_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        # self.thresh_image = cv2.threshold(self.blur_image, 45, 255, cv2.THRESH_BINARY)[1]
        self.morph_image = self.morph_transformation(self.thresh_image)
        # self.canny_image = cv2.Canny(self.thresh_image, threshold1=120, threshold2=255, edges=1)
        self.canny_image = self.edge_detection(self.morph_image)
        
    def morph_transformation(self, frame):
        kernel = np.ones((5,5),np.uint8)
        # opening = cv2.morphologyEx(frame, cv2.MORPH_CLOSE, kernel)
        opening = cv2.morphologyEx(frame, cv2.MORPH_OPEN, kernel)
        return opening
    
    def edge_detection(self, frame, sigma=0.33):
        # compute the median of the single channel pixel intensities
        v = np.median(frame)
        
        # apply automatic Canny edge detection using the computed median
        lower = int(max(0, (1.0 - sigma) * v))
        upper = int(min(255, (1.0 + sigma) * v))
        edged = cv2.Canny(frame, lower, upper)
        
        return edged
    
    def feature_detection(self):
        for img in self.cont_imgs:
            gray = np.float32(img)
            corners = cv2.goodFeaturesToTrack(gray, 8, 0.01, 50)
            if corners is not None:
                corners = np.int0(corners).reshape(-1, 2)
                corners = corners[np.argsort(corners[:, 0])]
                
                for i in corners:
                    x, y = i.ravel()
                    cv2.circle(self.org_image, (x, y), 4, 200, -1)
                        
    def get_waypoints(self):
        pass
    
    def show_image(self):
        cv2.imshow("edgedImg", self.canny_image)
        # cv2.imshow("hullImg", self.hull_img)
        cv2.imshow('frame', self.org_image)
        cv2.waitKey(1)


if __name__ == "__main__":
    it = ImageTracer()
    it.grab_image()