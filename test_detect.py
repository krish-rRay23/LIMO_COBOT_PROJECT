from ultralytics import YOLO
import cv2

# Load the model Roboflow provided (change this if Roboflow gave another filename)
model = YOLO("runs/detect/train/weights/best.pt")

# Load the test image
img = cv2.imread("your_image.png")  # change to your filename

# Run inference
results = model.predict(source=img, show=True, conf=0.5)

# Optional: Save the result
results[0].save(filename="prediction.jpg")

