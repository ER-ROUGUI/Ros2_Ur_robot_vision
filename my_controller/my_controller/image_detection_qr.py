import cv2
import numpy as np
class QRDetector:
    def __init__(self, camera_url="http://192.168.1.102:4242/current.jpg?annotations=on/off", fx=679.9691, fy=679.8200, u0=304.6999, v0=238.7130):
        self.camera_url = camera_url
        self.fx = fx
        self.fy = fy
        self.u0 = u0
        self.v0 = v0
        self.cap = cv2.VideoCapture(self.camera_url)

    def get_image_from_camera(self):
        try:
            ret, img = self.cap.read()
            if ret:
                return img
            else:
                return None
        except Exception as e:
            print(f"Error during frame capture: {e}")
            return None
        
    def detect_qr_and_display(self, img):
        if img is not None:
            ret_qr, decoded_info, points, _ = cv2.QRCodeDetector().detectAndDecodeMulti(img)

            if ret_qr:
                for s, p in zip(decoded_info, points):
                    if s:
                        print(s)
                        color = (0, 255, 0)
                    else:
                        color = (0, 0, 255)

                    qr_coordinates = []
                    for points in p :
                        qr_coordinates.append((int(points[0]), int(points[1])))

                        img = cv2.circle(img, (int(points[0]),int(points[1])), radius=8, color=(0, 0, 255), thickness=-1)
                
                #cv2.imshow('QR Code Detection', img)
                return qr_coordinates
            else:
                return []
        else:
            return []
    
    def normalized_coordinates(self, x_pixel, y_pixel):
        x_normalized = (x_pixel - self.u0) / self.fx
        y_normalized = (y_pixel - self.v0) / self.fy
        return x_normalized, y_normalized

    #computing error with pixel cordinates
    # def calculate_error(self, img, fixed_points, qr_coordinates):
    #     if not fixed_points or not qr_coordinates:
    #         return np.zeros(2 * len(fixed_points))

    #     # Initialize errors vector
    #     errors = np.zeros(2 * len(fixed_points))

    #     for i, point in enumerate(fixed_points):
    #         # Use pixel coordinates for error calculation
    #         errors[2 * i], errors[2 * i + 1] = point[0] - qr_coordinates[i][0], point[1] - qr_coordinates[i][1]
    
                
    #         # Draw line between fixed point and corresponding QR code point
    #         cv2.line(img, point, (int(qr_coordinates[i][0]), int(qr_coordinates[i][1])), (255, 0, 0), 2)

    #     return errors
    
    # comput error with normalized cordinates
    
    def calculate_error(self, img, fixed_points, qr_coordinates):
        if not fixed_points or not qr_coordinates:
            return np.zeros(2 * len(fixed_points))

        # Initialize errors vector
        errors = np.zeros(2 * len(fixed_points))

        for i, point in enumerate(fixed_points):

            fixed_point_normalized = self.normalized_coordinates(point[0], point[1])
            qr_point_normalized = self.normalized_coordinates(qr_coordinates[i][0], qr_coordinates[i][1])
            errors[2 * i], errors[2 * i + 1] = fixed_point_normalized[0] - qr_point_normalized[0], fixed_point_normalized[1] - qr_point_normalized[1]
            # Draw line between fixed point and corresponding QR code point
            cv2.line(img, point, (int(qr_coordinates[i][0]), int(qr_coordinates[i][1])), (255, 0, 0), 2)

        return errors

    def display_camera_stream(self):
        while True:
            img = self.get_image_from_camera()
            if img is not None:
                qr_coordinates = self.detect_qr_and_display(img)

                # Afficher les points fixes
                fixed_points = [
                    (100, 100),
                    (500, 100),
                    (500, 400),
                    (100,400)
                ]
                for point in fixed_points:
                    cv2.circle(img, point, 5, (255, 0, 0), -1)

                errors = self.calculate_error(img,fixed_points, qr_coordinates)
                # print(np.shape(errors))
                print("Error Vector:", errors)
       
                cv2.imshow('Camera Stream', img)

            # Press 'q' to exit the stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    circle_detector = QRDetector(camera_url=0) # use camera_urk = "your url"
    circle_detector.display_camera_stream()
