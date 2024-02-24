import cv2
import requests
import numpy as np

class CircleDetector:
    def __init__(self, camera_url= "http://192.168.1.102:4242/current.jpg?annotations=on/off", param1=50, param2=30, min_radius=10, max_radius=50, num_circles=3,fx=679.9691, fy=679.8200, u0=304.6999, v0=238.7130):
        # 
        self.camera_url = camera_url
        self.param1 = param1
        self.param2 = param2
        self.min_radius = min_radius
        self.max_radius = max_radius
        self.num_circles = num_circles
        self.fx = fx
        self.fy = fy
        self.u0 = u0
        self.v0 = v0
        self.cap = cv2.VideoCapture(self.camera_url)

    # def get_image_from_camera(self):
    #     try:
    #         response = requests.get(self.camera_url)
    #         img_array = np.array(bytearray(response.content), dtype=np.uint8)
    #         img = cv2.imdecode(img_array, -1)
            
    #         return img
    #     except Exception as e:
    #         #print(f"Erreur lors de la récupération de l'image: {e}")
    #         return None
        
    def get_image_from_camera(self):
        try:
            # Capture a frame from the camera (assuming camera index is specified in camera_url)
            ret, img = self.cap.read()

            if ret:
                return img
            else:
                return None
        except Exception as e:
            # Handle any exceptions that might occur during frame capture
            # For example, camera not available or an error during capture
            print(f"Error during frame capture: {e}")
            return None

    def detect_circles_and_display(self, img):
        if img is not None:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            gray_blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            circles = cv2.HoughCircles(
                gray_blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=20,
                param1=self.param1,
                param2=self.param2,
                minRadius=self.min_radius,
                maxRadius=self.max_radius
            )

            if circles is not None:
                circles = np.uint16(np.around(circles))[0, :]
                circles = sorted(circles, key=lambda x: x[2], reverse=True)[:self.num_circles]

                # Draw circles on the image
                for i, circle in enumerate(circles):
                    center = (circle[0], circle[1])
                    radius = circle[2]

                    cv2.circle(img, center, radius, (0, 255, 0), 2)
                    #print(f"Position du cercle {i+1} - x: {center[0]}, y: {center[1]}, rayon: {radius}")

                return circles
            else:
                return []
        else:
            return []

    def normalized_coordinates(self, x_pixel, y_pixel):
        x_normalized = (x_pixel-self.u0)/(self.fx )
        y_normalized = (y_pixel-self.v0)/(self.fy )
        return x_normalized, y_normalized
    
    def calculate_error(self, img, fixed_points, circles):
        if not fixed_points or not circles:
            return np.zeros(2 * len(fixed_points))

        # Initialize errors vector
        errors = np.zeros(2 * len(fixed_points))

        # Iterate over each fixed point and find the corresponding circle
        for i, point in enumerate(fixed_points):
            min_distance = float('inf')
            closest_circle_index = -1

            for j, circle in enumerate(circles):
                # Use normalized coordinates for distance calculation
                normalized_point = self.normalized_coordinates(point[0], point[1])
                normalized_circle = self.normalized_coordinates(circle[0], circle[1])

                distance = np.linalg.norm(np.array(normalized_point) - np.array(normalized_circle))

                if distance < min_distance:
                    min_distance = distance
                    closest_circle_index = j

            if closest_circle_index != -1:
                # Compute error in normalized coordinates
                errors[2 * i], errors[2 * i + 1] = self.normalized_coordinates(point[0] - circles[closest_circle_index][0],
                                                                                point[1] - circles[closest_circle_index][1])

                # Draw line between fixed point and its corresponding circle center
                cv2.line(img, point, (int(circles[closest_circle_index][0]), int(circles[closest_circle_index][1])), (255, 0, 0), 2)

        return errors


    def display_camera_stream(self):
        while True:
            img = self.get_image_from_camera()
            if img is not None:
                circles = self.detect_circles_and_display(img)

                # Afficher les points fixes
                fixed_points = [
                    (100, 100),
                    (500, 100),
                    (300, 400)
                ]
                for point in fixed_points:
                    cv2.circle(img, point, 5, (255, 0, 0), -1)

                errors = self.calculate_error(img,fixed_points, circles)
                # print(np.shape(errors))
                print("Error Vector:", errors)
                # return errors

                # Afficher l'image
                cv2.imshow('Camera Stream', img)

            # Press 'q' to exit the stream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Release the camera when the stream is closed
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    circle_detector = CircleDetector(camera_url=0)
    circle_detector.display_camera_stream()
