import cv2
import os
import time

# Camera URL
camera_url = "http://192.168.1.102:4242/current.jpg?annotations=on/off"

num = 6

# Create a directory to store images if it doesn't exist
output_folder = "images2"
os.makedirs(output_folder, exist_ok=True)

start_time = time.time()
capture_interval = 4.0  # Capture every 6 seconds

# Open a connection to the camera using the URL
cap = cv2.VideoCapture(camera_url)

while True:
    elapsed_time = time.time() - start_time

    if elapsed_time >= capture_interval:
        try:
            # Capture a frame from the camera
            ret, img = cap.read()

            if not ret:
                print("Error: Unable to capture frame.")
                break

            cv2.imshow('Img 1', img)

            # Save the frame to the images folder
            cv2.imwrite(os.path.join(output_folder, f"image{num}.png"), img)
            print(f"Image {num} saved!")

            num += 1
            start_time = time.time()

        except Exception as e:
            print(f"Error: {e}")

    k = cv2.waitKey(5)

    if k == 27:  # Press 'Esc' key to exit
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()
