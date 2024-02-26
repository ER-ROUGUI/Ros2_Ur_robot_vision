
import numpy as np

fixed_points = [
                    (100, 100),
                    (500, 100),
                    (500, 400),
                    (100,400)
                ]

qr_coordinates = [
                    (300, 170),
                    (252, 100),
                    (185, 167),
                    (120,288)
                ]


def normalized_coordinates( x_pixel, y_pixel):
        x_normalized = (x_pixel - 304.6999) / 679.9691
        y_normalized = (y_pixel - 238.7130) / 679.8200
        return x_normalized, y_normalized

def calculate_error(fixed_points, qr_coordinates):
        if not fixed_points or not qr_coordinates:
            return np.zeros(2 * len(fixed_points))

        # Initialize errors vector
        errors = np.zeros(2 * len(fixed_points))

        for i, point in enumerate(fixed_points):
            # Use pixel coordinates for error calculation
            # errors[2 * i], errors[2 * i + 1] = point[0] - qr_coordinates[i][0], point[1] - qr_coordinates[i][1]
            # Use metres coordinates for error calculation
            fixed_point_normalized = normalized_coordinates(point[0], point[1])
            qr_point_normalized = normalized_coordinates(qr_coordinates[i][0], qr_coordinates[i][1])
            errors[2 * i], errors[2 * i + 1] = fixed_point_normalized[0] - qr_point_normalized[0], fixed_point_normalized[1] - qr_point_normalized[1]
            
            # Draw line between fixed point and corresponding QR code point
            # cv2.line(img, point, (int(qr_coordinates[i][0]), int(qr_coordinates[i][1])), (255, 0, 0), 2)
            
        return errors

print(calculate_error(fixed_points , qr_coordinates))
# print(normalized_coordinates(fixed_points[0][0],fixed_points[0][1]))
# print(calculate_error(fixed_points,qr_coordinates))