"""
Detects the board and x piece and marks them:
- Green dot: board 
- Red dot: piece

Requirements:
- Python 3.x
- OpenCV (`cv2`)
- NumPy

Install dependencies:
    python3 -m pip install opencv-python numpy

Usage:
1. Place 'image.jpeg' in the script directory.
2. Adjust crop coordinates (x1, x2, y1, y2) if needed.
3. Run:
    python3 piece_detection.py
4. Output saved as 'filtered_objects.jpeg'.
"""

import cv2
import numpy as np

image = cv2.imread('image.jpeg')

if image is None:
    print("Could not load image.")
    exit()

# Crop
x1, x2 = 350, 914 # Might need to change these for the
y1, y2 = 150, 686 # actual camera (might be different for different robots)
image = image[y1:y2, x1:x2]

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Define black range + mask
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 50])

mask = cv2.inRange(hsv, lower_black, upper_black)
mask = cv2.medianBlur(mask, 5)

# Find contours of black objects
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter out small objects
min_area = 500 
filtered_mask = np.zeros_like(mask)
for cnt in contours:
    if cv2.contourArea(cnt) >= min_area:
        cv2.drawContours(filtered_mask, [cnt], -1, 255, -1)

# Keep large black objects
result = np.where(filtered_mask[:, :, None].astype(bool), image, np.ones_like(image) * 255)

# Loop through found black objects
for cnt in contours:
    if cv2.contourArea(cnt) < min_area:
        continue

    # Get centroid
    M = cv2.moments(cnt)
    if M["m00"] == 0:
        continue
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    x, y, w, h = cv2.boundingRect(cnt)
    object_pixels = image[y:y+h, x:x+w]
    print(f"Object at ({cx},{cy}), area={cv2.contourArea(cnt)}, pixel values shape={object_pixels.shape}")

    if cv2.contourArea(cnt) > 5000:  
        cv2.circle(result, (cx, cy), 10, (0, 255, 0), -1)  # green for board
    else:
        cv2.circle(result, (cx, cy), 10, (0, 0, 255), -1)  # red for piece

cv2.imwrite("filtered_objects.jpeg", result)

# not for mac
# cv2.waitKey(0)
# cv2.destroyAllWindows()
