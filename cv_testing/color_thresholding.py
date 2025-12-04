import cv2
import numpy as np
import os

# A dummy function for the trackbars. It is required by cv2.createTrackbar().
def nothing(x):
    pass

# Specify the path to your image file.
# Replace 'your_image.jpg' with the name of your image.
image_path = 'test3.png'

# Check if the image exists before proceeding.
if not os.path.exists(image_path):
    print(f"Error: The image file '{image_path}' was not found.")
    print("Please make sure the image is in the same directory as the script or provide the full path.")
else:
    # Load the image
    frame = cv2.imread(image_path)
    
    # Create a window to display the original image and the masked image.
    cv2.namedWindow('Image')
    cv2.namedWindow('Masked Image')
    
    # Convert the loaded image to HSV color space.
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create trackbars for HSV values.
    # The maximum value for Hue in OpenCV is 179.
    cv2.createTrackbar('H_min', 'Image', 0, 179, nothing)
    cv2.createTrackbar('H_max', 'Image', 179, 179, nothing)
    cv2.createTrackbar('S_min', 'Image', 0, 255, nothing)
    cv2.createTrackbar('S_max', 'Image', 255, 255, nothing)
    cv2.createTrackbar('V_min', 'Image', 0, 255, nothing)
    cv2.createTrackbar('V_max', 'Image', 255, 255, nothing)
    
    while True:
        # Get the current trackbar positions.
        h_min = cv2.getTrackbarPos('H_min', 'Image')
        h_max = cv2.getTrackbarPos('H_max', 'Image')
        s_min = cv2.getTrackbarPos('S_min', 'Image')
        s_max = cv2.getTrackbarPos('S_max', 'Image')
        v_min = cv2.getTrackbarPos('V_min', 'Image')
        v_max = cv2.getTrackbarPos('V_max', 'Image')
    
        # Create NumPy arrays for the lower and upper HSV thresholds.
        lower_bound = np.array([h_min, s_min, v_min])
        upper_bound = np.array([h_max, s_max, v_max])
    
        # Create a mask using cv2.inRange().
        # This will create a binary mask where pixels within the HSV range are white.
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)
    
        # Apply the mask to the original image using a bitwise AND operation.
        result = cv2.bitwise_and(frame, frame, mask=mask)
    
        # Display the images.
        cv2.imshow('Image', frame)
        cv2.imshow('Masked Image', result)
    
        # Break the loop if the 'q' key is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    # Destroy all windows and release resources.
    cv2.destroyAllWindows()


