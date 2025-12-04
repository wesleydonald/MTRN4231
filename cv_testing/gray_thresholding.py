import cv2
import numpy as np
import os

# A dummy function for the trackbars. It is required by cv2.createTrackbar().
def nothing(x):
    pass

# --- Setup ---
# Specify the path to your image file.
# Replace 'your_image.jpg' with the name of your image.
image_path = 'test.png'

# Check if the image exists before proceeding.
if not os.path.exists(image_path):
    print(f"Error: The image file '{image_path}' was not found.")
    print("Please make sure the image is in the same directory as the script or provide the full path.")
else:
    # Load the image in grayscale from the start.
    # The cv2.IMREAD_GRAYSCALE flag ensures the image is loaded as a single channel.
    original_image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Check if the image was loaded correctly.
    if original_image is None:
        print(f"Error: Could not read the image file '{image_path}'.")
    else:
        # Create a window to display the output.
        cv2.namedWindow('Thresholding App')

        # Create a trackbar for the threshold value.
        # The value can range from 0 to 255, the standard range for grayscale images.
        cv2.createTrackbar('Threshold', 'Thresholding App', 127, 255, nothing)
        
        while True:
            # Get the current trackbar position.
            threshold_value = cv2.getTrackbarPos('Threshold', 'Thresholding App')
            
            # Apply a binary threshold to the grayscale image.
            # Pixels with intensity greater than the threshold_value become white (255),
            # and all others become black (0).
            _, thresholded_image = cv2.threshold(original_image, threshold_value, 255, cv2.THRESH_BINARY)
            
            # Display the images.
            cv2.imshow('Thresholding App', thresholded_image)
            cv2.imshow('Original Grayscale Image', original_image)
            
            # Break the loop if the 'q' key is pressed.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        # Destroy all windows and release resources.
        cv2.destroyAllWindows()


